import argparse
from pathlib import Path

from live_tracking import run_live_tracking


def main():
    project_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="水池雷达与三摄像头实时联动系统")
    parser.add_argument(
        "--config",
        default=str(project_dir / "live_config.json"),
        help="实时联动配置文件，默认使用程序目录中的 live_config.json",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("水池雷达与三摄像头实时联动系统")
    print("按 Ctrl+C 停止程序")
    print("=" * 60)
    run_live_tracking(args.config)


if __name__ == "__main__":
    main()
