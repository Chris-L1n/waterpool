import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request
import uuid


MS_PER_DEGREE_DEFAULT = 26.4
DEFAULT_ACTION = "move_right"
DEFAULT_CAMERA = "A"
DEFAULT_MOVE_SPEED = 8
DEFAULT_MOVE_DEGREES = 30.0
DEFAULT_LIMIT_DEGREES = 360.0
DEFAULT_HOME_DEGREES = 180.0
DEFAULT_SETTLE_S = 0.5


class ImouAPIError(RuntimeError):
    pass


class ImouPTZClient:
    """Small thread-safe client for accessToken and controlPTZ."""

    def __init__(
        self,
        base_url,
        app_id,
        app_secret,
        cameras,
        timeout_s=20,
        ms_per_degree=MS_PER_DEGREE_DEFAULT,
        verbose=False,
        access_token=None,
    ):
        self.base_url = base_url.rstrip("/")
        self.app_id = app_id
        self.app_secret = app_secret
        self.cameras = cameras
        self.timeout_s = float(timeout_s)
        self.ms_per_degree = float(ms_per_degree)
        self.verbose = bool(verbose)
        self._token = access_token
        self._token_deadline = time.time() + 5400 if access_token else 0.0
        self._lock = threading.Lock()

    def _system(self):
        timestamp = str(int(time.time()))
        nonce = str(uuid.uuid4())
        sign_text = f"time:{timestamp},nonce:{nonce},appSecret:{self.app_secret}"
        return {
            "ver": "1.0",
            "appId": self.app_id,
            "sign": hashlib.md5(sign_text.encode("utf-8")).hexdigest().lower(),
            "time": int(timestamp),
            "nonce": nonce,
        }

    def _post(self, method, params):
        body = {
            "system": self._system(),
            "id": str(uuid.uuid4()),
            "params": params,
        }
        if self.verbose:
            print("\nREQUEST BODY:")
            print(json.dumps(body, ensure_ascii=False, indent=2))

        request = urllib.request.Request(
            f"{self.base_url}/{method}",
            data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                result = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise ImouAPIError(f"HTTP {exc.code}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise ImouAPIError(f"Network error: {exc}") from exc

        if self.verbose:
            print("\nRESPONSE:")
            print(json.dumps(result, ensure_ascii=False, indent=2))

        code = result.get("result", {}).get("code")
        if code != "0":
            msg = result.get("result", {}).get("msg", "unknown error")
            raise ImouAPIError(f"{method} failed: {code} {msg}")
        return result

    def access_token(self, force=False):
        with self._lock:
            if not force and self._token and time.time() < self._token_deadline:
                return self._token
            result = self._post("accessToken", {})
            data = result["result"]["data"]
            self._token = data["accessToken"]
            # The service normally returns a long-lived token. Refresh early.
            self._token_deadline = time.time() + 5400
            return self._token

    def degrees_to_duration_ms(self, degrees):
        if float(degrees) < 0:
            raise ValueError("degrees must be greater than or equal to 0.")
        return max(1, int(round(float(degrees) * self.ms_per_degree)))

    def move(self, camera_key, h=0, v=0, z=1, duration_ms=100):
        camera = self.cameras[camera_key]
        params = {
            "token": self.access_token(),
            "deviceId": camera["device_id"],
            "channelId": str(camera.get("channel_id", "0")),
            "operation": "move",
            "h": float(h),
            "v": float(v),
            "z": float(z),
            "duration": str(duration_ms),
        }
        return self._post("controlPTZ", params)

    def move_horizontal(self, camera_key, direction, speed, duration_ms):
        h = abs(float(speed)) if direction == "right" else -abs(float(speed))
        return self.move(camera_key, h=h, v=0, z=1, duration_ms=duration_ms)

    def move_vertical(self, camera_key, direction, speed, duration_ms):
        v = abs(float(speed)) if direction == "up" else -abs(float(speed))
        return self.move(camera_key, h=0, v=v, z=1, duration_ms=duration_ms)

    def stop(self, camera_key):
        return self.move(camera_key, h=0, v=0, z=1, duration_ms=100)

    def move_horizontal_degrees(self, camera_key, direction, speed, degrees):
        duration_ms = self.degrees_to_duration_ms(degrees)
        print(
            f"Move {direction} once: camera={camera_key}, degrees={degrees}, "
            f"duration={duration_ms} ms, speed={speed}."
        )
        result = self.move_horizontal(camera_key, direction, speed, duration_ms)
        time.sleep(duration_ms / 1000.0 + 0.3)
        return result

    def move_vertical_degrees(self, camera_key, direction, speed, degrees):
        duration_ms = self.degrees_to_duration_ms(degrees)
        print(
            f"Move {direction} once: camera={camera_key}, degrees={degrees}, "
            f"duration={duration_ms} ms, speed={speed}."
        )
        self.stop(camera_key)
        time.sleep(0.2)
        result = self.move_vertical(camera_key, direction, speed, "last")
        time.sleep(2.0)
        self.stop(camera_key)
        time.sleep(0.3)
        return result

    def move_until_stop(self, camera_key, direction, speed, wait_degrees, settle_s=DEFAULT_SETTLE_S):
        wait_ms = self.degrees_to_duration_ms(wait_degrees)
        print(
            f"Move {direction} continuously: camera={camera_key}, duration='last', "
            f"target_wait_degrees={wait_degrees}, local_wait={wait_ms} ms, speed={speed}."
        )
        result = self.move_horizontal(camera_key, direction, speed, "last")
        time.sleep(wait_ms / 1000.0)
        self.stop(camera_key)
        time.sleep(float(settle_s))
        return result

    def approximate_home_from_left(self, camera_key, speed, limit_degrees=360, home_degrees=180, settle_s=0.5):
        """Approximate home because this model does not support operation=locate."""
        print("home is approximate because this camera does not support operation=locate.")
        print(
            f"Step 1/2: move left continuously for about {limit_degrees} degrees "
            "to reach the left reference position."
        )
        self.move_until_stop(camera_key, "left", speed, limit_degrees, settle_s=settle_s)
        time.sleep(float(settle_s))

        print(f"Step 2/2: move right by {home_degrees} degrees once to return to the preset position.")
        return self.move_horizontal_degrees(camera_key, "right", speed, home_degrees)

    def run_action(
        self,
        camera_key,
        action,
        speed=DEFAULT_MOVE_SPEED,
        degrees=DEFAULT_MOVE_DEGREES,
        limit_degrees=DEFAULT_LIMIT_DEGREES,
        home_degrees=DEFAULT_HOME_DEGREES,
        settle_s=DEFAULT_SETTLE_S,
    ):
        speed = abs(float(speed))

        if action == "move_right":
            return self.move_horizontal_degrees(camera_key, "right", speed, degrees)
        if action == "move_left":
            return self.move_horizontal_degrees(camera_key, "left", speed, degrees)
        if action == "move_up":
            return self.move_vertical_degrees(camera_key, "up", speed, degrees)
        if action == "move_down":
            return self.move_vertical_degrees(camera_key, "down", speed, degrees)
        if action == "right_limit":
            return self.move_until_stop(camera_key, "right", speed, limit_degrees, settle_s=settle_s)
        if action == "left_limit":
            return self.move_until_stop(camera_key, "left", speed, limit_degrees, settle_s=settle_s)
        if action == "home":
            return self.approximate_home_from_left(
                camera_key,
                speed,
                limit_degrees=limit_degrees,
                home_degrees=home_degrees,
                settle_s=settle_s,
            )
        if action == "stop":
            return self.stop(camera_key)
        raise ValueError(f"Unknown action: {action}")


def load_config(path):
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


def build_client_from_config(config, verbose=False, access_token=None):
    tracking = config.get("tracking", {})
    api = config["imou"]
    token = access_token or api.get("access_token")
    cameras = {
        item["key"]: item
        for item in config["cameras"]
        if item.get("enabled", True)
    }
    return ImouPTZClient(
        api["base_url"],
        api["app_id"],
        api["app_secret"],
        cameras,
        ms_per_degree=float(tracking.get("ms_per_degree", MS_PER_DEGREE_DEFAULT)),
        verbose=verbose,
        access_token=token,
    )


def parse_args():
    project_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Imou PTZ control for direct testing and radar-link integration."
    )
    parser.add_argument(
        "--config",
        default=str(project_dir / "live_config.json"),
        help="Path to live_config.json.",
    )
    parser.add_argument(
        "--camera",
        default=os.getenv("IMOU_CAMERA", DEFAULT_CAMERA),
        help="Camera key in live_config.json, for example A/B/C.",
    )
    parser.add_argument(
        "--action",
        choices=[
            "move_left",
            "move_right",
            "move_up",
            "move_down",
            "left_limit",
            "right_limit",
            "home",
            "stop",
        ],
        default=os.getenv("IMOU_PTZ_ACTION", DEFAULT_ACTION),
    )
    parser.add_argument("--speed", type=float, default=DEFAULT_MOVE_SPEED)
    parser.add_argument("--degrees", type=float, default=DEFAULT_MOVE_DEGREES)
    parser.add_argument("--limit-degrees", type=float, default=DEFAULT_LIMIT_DEGREES)
    parser.add_argument("--home-degrees", type=float, default=DEFAULT_HOME_DEGREES)
    parser.add_argument("--settle-s", type=float, default=DEFAULT_SETTLE_S)
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print OpenAPI request and response bodies.",
    )
    parser.add_argument(
        "--access-token",
        default=os.getenv("IMOU_ACCESS_TOKEN"),
        help="Use an existing accessToken and skip the accessToken request.",
    )
    args = parser.parse_args()

    if args.speed < 1 or args.speed > 8:
        raise SystemExit("--speed must be between 1 and 8.")
    if args.degrees < 0 or args.limit_degrees < 0 or args.home_degrees < 0:
        raise SystemExit(
            "--degrees, --limit-degrees, and --home-degrees must be greater than or equal to 0."
        )
    if args.settle_s < 0:
        raise SystemExit("--settle-s must be greater than or equal to 0.")
    return args


def main():
    args = parse_args()
    config = load_config(args.config)
    client = build_client_from_config(config, verbose=args.verbose, access_token=args.access_token)

    if args.camera not in client.cameras:
        available = ", ".join(sorted(client.cameras))
        raise SystemExit(f"Unknown camera '{args.camera}'. Available cameras: {available}")

    camera = client.cameras[args.camera]
    print(f"Config: {Path(args.config).resolve()}")
    print(f"Camera: {args.camera} / {camera['device_id']}")
    print(f"Action: {args.action}")
    client.run_action(
        camera_key=args.camera,
        action=args.action,
        speed=args.speed,
        degrees=args.degrees,
        limit_degrees=args.limit_degrees,
        home_degrees=args.home_degrees,
        settle_s=args.settle_s,
    )
    print("\nDone.")


if __name__ == "__main__":
    main()
