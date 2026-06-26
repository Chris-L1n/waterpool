import ctypes as ct
import socket
import struct
import time

"""
封装radar_track_parser_v4.py
提供给live_tracking.py 调用

"""
from anomaly_detector import AnomalyDetector
from boat_target_filter import BoatTargetSelector
from radar_track_parser_v4 import (
    CsvWriters,
    RadarReassembler,
    SimpleTracker,
    VCI_CAN_OBJ,
    VCI_USBCAN2,
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
        csv_cfg = cfg.get("csv", {})

        dll = load_dll()
        set_prototypes(dll)
        tracker = SimpleTracker(
            match_threshold_m=float(cfg.get("track_threshold_m", 0.4)),
            max_missed=int(cfg.get("max_missed", 3)),
        )
        boat_selector = BoatTargetSelector(
            cfg.get("boat_filter", {}),
        )
        writers = CsvWriters(
            csv_cfg.get("targets", "radar_targets.csv"),
            csv_cfg.get("nearest", "radar_nearest.csv"),
            csv_cfg.get("tracks", "radar_tracks.csv"),
            csv_cfg.get("filtered", "radar_filtered.csv"),
        )
        packet_no = 0

        # VOFA+ UDP 转发（开关由配置控制，默认关闭）
        vofa_cfg = cfg.get("vofa", {})
        vofa_enabled = bool(vofa_cfg.get("enabled", False))
        vofa_sock = None
        vofa_protocol = vofa_cfg.get("protocol", "justfloat")
        if vofa_enabled:
            vofa_host = vofa_cfg.get("host", "127.0.0.1")
            vofa_port = int(vofa_cfg.get("port", 1347))
            vofa_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"VOFA+ forwarding: {vofa_protocol} -> {vofa_host}:{vofa_port}")

        def on_packet(packet, timestamp):
            nonlocal packet_no
            targets, error = parse_target_packet(packet)
            if error or targets is None:
                return
            packet_no += 1
            targets = tracker.update(targets, packet_no)

            # ① BoatTargetSelector.select()：填充 first_position + track_hits（必须每帧调用）
            nearest = boat_selector.select(targets, packet_no)

            # ② 过滤噪声（含位移过滤，此时 first_position 已填充）
            boat_targets = [t for t in targets if boat_selector._passes_filters(t)]
            boat_targets = [t for t in boat_targets if boat_selector._passes_displacement(t)]

            # ③ 异常检测：对过滤后的目标做 SC/KS/KA
            if self.anomaly_detector is not None:
                self.anomaly_detector.feed(boat_targets)
                anomaly_active = self.anomaly_detector.is_active()
            else:
                anomaly_active = False

            writers.write_targets(packet_no, targets)
            writers.write_filtered(packet_no, boat_targets)
            # 异常活跃期间持续发目标给摄像头，而非仅触发瞬间动一次
            if anomaly_active and nearest is not None:
                writers.write_nearest(packet_no, nearest)
                self.on_nearest(nearest, boat_targets)
            else:
                writers.write_nearest(packet_no, None)

            # VOFA+ UDP 转发：只发过滤后的目标（看到的才是算法认为的"船"）
            if vofa_sock is not None:
                try:
                    for t in boat_targets:
                        buf = struct.pack('<ffff',
                            float(t['x_m']),
                            float(t['y_m']),
                            float(t['speed_m_s']),
                            float(t.get('pv', 0)))
                        buf += b'\x00\x00\x80\x7F'
                        vofa_sock.sendto(buf, (vofa_host, vofa_port))
                except Exception:
                    pass

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
            can_frame_no = 0
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
                    can_frame_no += count
                    for index in range(count):
                        frame = frames[index]
                        data = bytes(frame.Data[: int(frame.DataLen)])
                        reassembler.feed(int(frame.ID), data, int(frame.TimeStamp))
                    if can_frame_no % 500 == 0:
                        print(f"[CAN] 已收到 {can_frame_no} 帧, 完整包 {packet_no}")
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
