import ctypes as ct
import time

"""
封装radar_track_parser_v4.py
提供给live_tracking.py 调用

"""
from anomaly_detector import AnomalyDetector
from radar_track_parser_v4 import (
    CsvWriters,
    RadarReassembler,
    SimpleTracker,
    VCI_CAN_OBJ,
    VCI_USBCAN2,
    choose_nearest,
    close_can,
    load_dll,
    open_can,
    parse_target_packet,
    set_prototypes,
)


class RadarLiveStream:
    """Reusable CAN receive loop built from radar_track_parser_v4.py."""

    def __init__(self, config, on_nearest, anomaly_config=None, on_anomaly=None):
        self.config = config
        self.on_nearest = on_nearest
        self.on_anomaly = on_anomaly
        self.running = False

        # 异常检测器（由 live_tracking.py 传入配置和回调）
        if anomaly_config and anomaly_config.get("anomaly_detection", {}).get("enabled"):
            self.anomaly_detector = AnomalyDetector(anomaly_config, on_anomaly=on_anomaly)
        else:
            self.anomaly_detector = None

    def stop(self):
        self.running = False

    def run(self):
        cfg = self.config
        channel = int(cfg.get("channel", 0))
        baud = int(cfg.get("baud", 500))
        batch = int(cfg.get("batch", 2500))
        wait_ms = int(cfg.get("wait_ms", 20))
        filters = cfg.get("filters", {})
        csv_cfg = cfg.get("csv", {})

        dll = load_dll()
        set_prototypes(dll)
        tracker = SimpleTracker(
            match_threshold_m=float(cfg.get("track_threshold_m", 0.4)),
            max_missed=int(cfg.get("max_missed", 3)),
        )
        writers = CsvWriters(
            csv_cfg.get("targets", "radar_targets.csv"),
            csv_cfg.get("nearest", "radar_nearest.csv"),
            csv_cfg.get("tracks", "radar_tracks.csv"),
        )
        packet_no = 0

        def on_packet(packet, timestamp):
            nonlocal packet_no
            targets, error = parse_target_packet(packet)
            if error or targets is None:
                return
            packet_no += 1
            targets = tracker.update(targets, packet_no)
            nearest = choose_nearest(
                targets,
                x_max_cm=int(filters.get("x_max_cm", 999999)),
                y_min_cm=int(filters.get("y_min_cm", 0)),
                speed_max_cm_s=int(filters.get("speed_max_cm_s", 999999)),
                enable_pv_filter=bool(filters.get("pv_filter", False)),
                pv_near_y_max_cm=int(filters.get("pv_near_y_max_cm", 1000)),
                pv_min_near=int(filters.get("pv_min_near", 40)),
                pv_mid_y_max_cm=int(filters.get("pv_mid_y_max_cm", 2000)),
                pv_min_mid=int(filters.get("pv_min_mid", 30)),
            )
            writers.write_targets(packet_no, targets)
            writers.write_nearest(packet_no, nearest)
            if self.anomaly_detector is not None:
                self.anomaly_detector.feed(targets)
            if nearest is not None:
                self.on_nearest(nearest, targets)

        reassembler = RadarReassembler(on_packet)
        array_type = VCI_CAN_OBJ * batch
        frames = array_type()
        opened = False

        try:
            opened = open_can(dll, channel, baud)
            if not opened:
                raise RuntimeError("Unable to open CAN adapter")
            self.running = True
            print(f"Radar live stream started: channel={channel}, baud={baud}K")
            while self.running:
                count = dll.VCI_Receive(
                    ct.c_uint32(VCI_USBCAN2),
                    ct.c_uint32(0),
                    ct.c_uint32(channel),
                    frames,
                    ct.c_uint32(batch),
                    ct.c_int(wait_ms),
                )
                if count and count > 0:
                    for index in range(count):
                        frame = frames[index]
                        data = bytes(frame.Data[: int(frame.DataLen)])
                        reassembler.feed(int(frame.ID), data, int(frame.TimeStamp))
                else:
                    time.sleep(0.01)
        finally:
            self.running = False
            if opened:
                close_can(dll)
            writers.close()
            if self.anomaly_detector is not None:
                self.anomaly_detector.close()
                print(f"异常检测结束: {self.anomaly_detector.summary()}")
