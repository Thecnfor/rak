#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立双摄像头预览：不依赖推理后端/机械臂，仅推流查看画面。

用法:
    python view_cam.py                                # 用 config_car.yml 里的 front/side 编号
    python view_cam.py --front 3 --side 4 --width 640 --height 480

启动后浏览器打开 http://<jetson-ip>:5000/ 查看 cam1(前置) / cam2(侧视)。
"""
import argparse
import os
import time

from smartcar import Camera, Streamer
from smartcar.whalesbot.tools import get_yaml

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_car.yml")


def load_camera_ids():
    """从 config_car.yml 读取摄像头编号，读不到时回落到 3/4。"""
    cfg = get_yaml(CONFIG_PATH) or {}
    cam = cfg.get("camera", {})
    return cam.get("front", 3), cam.get("side", 4)


def main():
    parser = argparse.ArgumentParser(description="双摄像头预览")
    parser.add_argument("--front", type=int, default=None,
                        help="前置摄像头编号(默认读 config_car.yml)")
    parser.add_argument("--side", type=int, default=None,
                        help="侧视摄像头编号(默认读 config_car.yml)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30)
    args = parser.parse_args()

    cfg_front, cfg_side = load_camera_ids()
    front = args.front if args.front is not None else cfg_front
    side = args.side if args.side is not None else cfg_side

    cap_front = Camera(front, args.width, args.height)
    cap_side = Camera(side, args.width, args.height)
    streamer = Streamer()  # 自动启动 http://<ip>:5000/

    delay = 1.0 / args.fps if args.fps > 0 else 0.033
    try:
        while True:
            streamer.update_frame(cap_front.read(), "cam1")
            streamer.update_frame(cap_side.read(), "cam2")
            time.sleep(delay)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在关闭...")
    finally:
        cap_front.close()
        cap_side.close()
        streamer.stop()
        print("已退出")


if __name__ == "__main__":
    main()
