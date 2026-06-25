import json
import math
import threading
import time
from pathlib import Path

from imou_ptz import ImouAPIError, ImouPTZClient
from radar_live import RadarLiveStream

"""
连接雷达数据和摄像头控制
"""


def normalize_angle(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


class LiveTrackingCoordinator:
    """Selects a camera and converts the latest radar target into PTZ steps."""

    def __init__(self, config):
        self.config = config
        tracking = config["tracking"]
        self.cameras = {item["key"]: item for item in config["cameras"] if item.get("enabled", True)}
        self.deadband_deg = float(tracking.get("deadband_deg", 3.0))
        self.max_step_deg = float(tracking.get("max_step_deg", 12.0))
        self.ms_per_degree = float(tracking.get("ms_per_degree", 26.4))
        self.speed = int(tracking.get("speed", 8))
        self.command_interval_s = float(tracking.get("command_interval_s", 1.2))
        self.handoff_cooldown_s = float(tracking.get("handoff_cooldown_s", 5.0))
        self.target_timeout_s = float(tracking.get("target_timeout_s", 2.0))
        self.dry_run = bool(tracking.get("dry_run", True))
        self.home_on_start = bool(tracking.get("home_on_start", False))
        self.home_on_exit = bool(tracking.get("home_on_exit", False))
        self.home_limit_degrees = float(tracking.get("home_limit_degrees", 360.0))
        self.home_degrees = float(tracking.get("home_degrees", 180.0))
        self.home_settle_s = float(tracking.get("home_settle_s", 0.5))

        api = config["imou"]
        self.client = ImouPTZClient(
            api["base_url"],
            api["app_id"],
            api["app_secret"],
            self.cameras,
            ms_per_degree=self.ms_per_degree,
        )
        self.current_pan = {
            key: float(camera.get("initial_pan_deg", 0.0))
            for key, camera in self.cameras.items()
        }
        self.active_camera = None
        self.last_switch_time = 0.0
        self.last_command_time = 0.0
        self._latest = None
        self._latest_lock = threading.Lock()
        self._running = False
        self._worker = None

    def start(self):
        if self.home_on_start:
            self.home_all("start")
        self._running = True
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

    def stop(self):
        self._running = False
        if self._worker:
            self._worker.join(timeout=3)
        if self.home_on_exit:
            self.home_all("exit")

    def home_all(self, reason="manual"):
        for key in sorted(self.cameras):
            message = (
                f"PTZ {key}: approximate home on {reason}, "
                f"left {self.home_limit_degrees:.1f}deg then right {self.home_degrees:.1f}deg"
            )
            try:
                if self.dry_run:
                    print("[DRY RUN]", message)
                else:
                    print(message)
                    self.client.approximate_home_from_left(
                        key,
                        speed=self.speed,
                        limit_degrees=self.home_limit_degrees,
                        home_degrees=self.home_degrees,
                        settle_s=self.home_settle_s,
                    )
                self.current_pan[key] = 0.0
            except ImouAPIError as exc:
                print(f"PTZ approximate home failed for camera {key}: {exc}")

    def submit(self, nearest, targets):
        with self._latest_lock:
            self._latest = (time.time(), dict(nearest), [dict(item) for item in targets])

    def on_anomaly(self, event):
        """异常检测回调 — 收到报警时打印，后续可扩展 PTZ 转向对应预置位"""
        print(f"[ANOMALY CALLBACK] {event['event_type']} {event['event_label']} "
              f"tracks={event['track_ids']}")
        # TODO: 根据 event_type 调用对应预置位
        # 例如: self.client.move_to_preset(key, preset_name)

    def _run_worker(self):
        while self._running:
            time.sleep(0.05)
            with self._latest_lock:
                latest = self._latest
            if latest is None:
                continue
            received_at, target, _ = latest
            if time.time() - received_at > self.target_timeout_s:
                continue
            if time.time() - self.last_command_time < self.command_interval_s:
                continue
            self._track(target)

    def _camera_distance(self, camera, target):
        cx, cy = camera["position_m"]
        return math.hypot(target["x_m"] - cx, target["y_m"] - cy)

    def _eligible(self, camera, target):
        distance = self._camera_distance(camera, target)
        return float(camera.get("min_range_m", 0.0)) <= distance <= float(camera["max_range_m"])

    def _select_camera(self, target):
        eligible = [camera for camera in self.cameras.values() if self._eligible(camera, target)]
        if not eligible:
            return None
        if self.active_camera:
            current = self.cameras[self.active_camera]
            if self._eligible(current, target):
                return current
        return min(eligible, key=lambda camera: self._camera_distance(camera, target))

    def _target_pan(self, camera, target):
        cx, cy = camera["position_m"]
        dx = target["x_m"] - cx
        dy = target["y_m"] - cy
        world_bearing = math.degrees(math.atan2(dx, dy))
        return normalize_angle(world_bearing - float(camera.get("heading_deg", 0.0)))

    def _track(self, target):
        camera = self._select_camera(target)
        if camera is None:
            print(f"No camera covers target x={target['x_m']:.2f}, y={target['y_m']:.2f}")
            self.last_command_time = time.time()
            return

        key = camera["key"]
        now = time.time()
        if self.active_camera != key:
            if self.active_camera and now - self.last_switch_time < self.handoff_cooldown_s:
                return
            print(f"Camera handoff: {self.active_camera or '-'} -> {key}")
            self.active_camera = key
            self.last_switch_time = now

        desired_pan = self._target_pan(camera, target)
        error = normalize_angle(desired_pan - self.current_pan[key])
        if abs(error) <= self.deadband_deg:
            self.last_command_time = now
            return

        step = max(-self.max_step_deg, min(self.max_step_deg, error))
        direction = "right" if step > 0 else "left"
        degrees = abs(step)
        duration_ms = max(1, round(degrees * self.ms_per_degree))
        message = (
            f"PTZ {key}: target=({target['x_m']:+.2f},{target['y_m']:.2f})m "
            f"pan={self.current_pan[key]:+.1f}->{desired_pan:+.1f}, "
            f"{direction} {degrees:.1f}deg/{duration_ms}ms"
        )

        try:
            if self.dry_run:
                print("[DRY RUN]", message)
            else:
                self.client.move_horizontal(key, direction, self.speed, duration_ms)
                print(message)
            self.current_pan[key] = normalize_angle(self.current_pan[key] + step)
        except ImouAPIError as exc:
            print(f"PTZ command failed for camera {key}: {exc}")
        finally:
            self.last_command_time = time.time()


def load_live_config(path):
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


def _make_output_dir(base_dir, ts):
    """创建带时间戳的输出目录：csv文件/20260626_143025/"""
    out = Path(base_dir) / "csv文件" / ts
    out.mkdir(parents=True, exist_ok=True)
    return out


def run_live_tracking(config_path):
    config_path = Path(config_path).resolve()
    config = load_live_config(config_path)

    # 生成本次运行的时间戳，创建输出文件夹
    from datetime import datetime
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = _make_output_dir(config_path.parent, ts)

    # 雷达 CSV 路径：csv文件/{ts}/radar_targets.csv
    csv_config = config.get("radar", {}).get("csv", {})
    for key, value in list(csv_config.items()):
        csv_config[key] = str(out_dir / Path(value).name)

    # 异常检测 CSV 路径
    ad_csv = config.get("anomaly_detection", {}).get("csv", "")
    config["anomaly_detection"]["csv"] = str(out_dir / Path(ad_csv).name)

    coordinator = LiveTrackingCoordinator(config)
    radar = RadarLiveStream(
        config["radar"],
        on_nearest=coordinator.submit,
        anomaly_config=config,
        on_anomaly=coordinator.on_anomaly,
    )
    coordinator.start()
    print(f"Live configuration: {config_path}")
    print(f"PTZ dry_run={coordinator.dry_run}")
    anomaly_enabled = config.get("anomaly_detection", {}).get("enabled", False)
    print(f"异常检测: {'启用' if anomaly_enabled else '关闭'}")
    print(f"输出目录: {out_dir}")
    try:
        radar.run()
    except KeyboardInterrupt:
        print("Stopping live tracking...")
    finally:
        radar.stop()
        coordinator.stop()
