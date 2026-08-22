#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""侧视(cam2)实时检测输出工具.

打开侧摄像头 + 任务推理后端, 实时打印画面中每个检测目标的
标签 / 置信度 / 归一化与像素 cx,cy,w,h. 只读检测, 不动车/臂.

数据来自后台实时检测线程写入的缓存 (tasks/tools/perception/realtime.py 的
get_realtime_detections), 每条检测结构:
    [cls_id, obj_id, label, score, nx, ny, nw, nh]
    nx,ny = 归一化中心(0~1, 左上角为原点)
    nw,nh = 归一化宽高(相对画面)

用法:
    python scripts/view_side_detections.py            # 默认后台缓存, 0.2s 刷新
    scripts/test_watering_.py    # 每帧同步跑一次推理(更即时但更占 GPU)
    python scripts/view_side_detections.py --label water   # 只看指定 label
    python scripts/view_side_detections.py --score 0.5     # 只显示置信度 >= 0.5
    python scripts/view_side_detections.py --hz 10         # 刷新频率(默认5)
退出: Ctrl+C
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tasks.tools import create_car


def fmt(d, frame_shape):
    """把一条检测格式化成可读字符串. d=[cls,obj,label,score,nx,ny,nw,nh]."""
    _, obj_id, label, score, nx, ny, nw, nh = d
    h, w = frame_shape[:2]
    px, py = nx * w, ny * h
    pw, ph = nw * w, nh * h
    return (
        f"{label:<14} score={score:.3f}  "
        f"norm cx={nx:.3f} cy={ny:.3f} w={nw:.3f} h={nh:.3f} | "
        f"px({px:5.1f},{py:5.1f}) pw={pw:5.1f} ph={ph:5.1f}"
    )


def main():
    ap = argparse.ArgumentParser(description="侧视实时检测输出")
    ap.add_argument("--fresh", action="store_true",
                    help="每帧同步跑一次推理(更即时); 默认读后台缓存")
    ap.add_argument("--label", default=None, help="只显示指定 label(如 water/cylinder_1)")
    ap.add_argument("--score", type=float, default=0.0, help="只显示置信度>=该值的目标")
    ap.add_argument("--hz", type=float, default=5.0, help="刷新频率(Hz), 默认5")
    ap.add_argument("--max-age", type=float, default=0.5,
                    help="后台缓存最大年龄(秒), 超龄视为无结果; 默认0.5")
    args = ap.parse_args()

    car = create_car(reset=False)  # 只起摄像头+推理+后台线程, 不复位臂/里程计
    interval = 1.0 / max(args.hz, 0.1)
    print("按 Ctrl+C 退出")
    try:
        while not getattr(car, "_stop_flag", False):
            t0 = time.time()
            dets = car.get_realtime_detections(
                fresh=args.fresh, max_age=args.max_age
            )
            shape = getattr(car.cap_side, "frame", None)
            frame_shape = shape.shape if shape is not None else (0, 0)

            rows = []
            for d in dets:
                if args.label is not None and d[2] != args.label:
                    continue
                if d[3] < args.score:
                    continue
                rows.append(fmt(d, frame_shape))
            # 控制台整块覆写(避免刷屏堆叠); 终端不支持则退化为逐行打印
            block = "\n".join(rows) if rows else "(无目标)"
            sys.stdout.write("\033[2J\033[H" + block + "\n")
            sys.stdout.flush()
            time.sleep(max(interval - (time.time() - t0), 0.01))
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        car.close()


if __name__ == "__main__":
    main()
