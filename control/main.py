from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import uvicorn

from .backend import TASK_ORDER
from .mock_backend import MockBackend
from .server import create_app


# =========================================================================
# 与 run.py 一致的任务结束钉姿势 + 里程计重置钩子
# =========================================================================
_TURN_LEFT_TASKS = {"target_detection", "watering"}
_TURN_LEFT_RAD = math.pi / 6

TASK_END_POSE = {
    "seeding": (-0.1, -0.05, "LEFT", "UP"),
    "target_detection": (-0.3, -0.05, "RIGHT", "UP"),
    "watering": (-0.0, -0.05, "LEFT", "UP"),
    "shooting": (-0.25, -0.2, "LEFT", "DOWN"),
    "harvesting": (-0.0, 0, "LEFT", "UP"),
    "sorting": (-0.3, -0.05, "RIGHT", "UP"),
    "ordering": (-0, 0, "LEFT", "UP"),
    "delivery": (-0, 0, "LEFT", "UP"),
}


def _pin_arm_and_reset(car, task_name: str) -> None:
    pose = TASK_END_POSE.get(task_name)
    if pose:
        x, y, arm, hand = pose
        print(f"[{task_name}] 钉机械臂位姿: x={x} y={y} arm={arm} hand={hand}")
        try:
            car.arm.set_arm_pose(x, y, arm, hand)
        except Exception as exc:
            print(f"[{task_name}] set_arm_pose 失败: {exc}")
    try:
        car.reset_position()
        car.get_odometry(True)
        car.get_distance(True)
        print(f"[{task_name}] 里程计已重置")
    except Exception as exc:
        print(f"[{task_name}] 里程计重置失败: {exc}")
    if task_name in _TURN_LEFT_TASKS:
        print(f"[{task_name}] 沿逆时针转 60° (起步巡线)")
        try:
            car.move_for(
                [0.0, 0.0, _TURN_LEFT_RAD],
                max_velocities=[0.10, 0.10, math.pi / 6],
            )
        except Exception as exc:
            print(f"[{task_name}] 起步调整朝向失败: {exc}")


def _build_real_backend(stream: bool = True):
    """构建真实硬件后端（延迟初始化：点"开始"才 create_car 占串口）。

    启动时不初始化硬件，只把配置传给 RealBackend；用户点"开始"时才
    _ensure_hardware() 初始化 create_car + Orchestrator，走 run.py 标准流程。
    """
    from .real_backend import RealBackend

    task_kwargs = {
        "shooting": {
            "animal_list": lambda results: results.get("target_detection"),
        },
        "delivery": {
            "order_list": lambda results: results.get("ordering"),
        },
    }
    backend = RealBackend(
        task_kwargs=task_kwargs,
        stream=stream,
        static_skip=set(),
        after_hook=_pin_arm_and_reset,
    )
    return backend, None


def _build_app(
    mode: str,
    stream: bool = True,
    host: str = "0.0.0.0",
    port: int = 8500,
):
    """根据模式构建 (app, backend, car_or_None)。"""
    car = None
    if mode == "real":
        print("[control] 构建真实后端（延迟初始化：点开始才占串口）")
        backend, car = _build_real_backend(stream=stream)
        print(f"[control] 后端就绪, 任务数 {len(TASK_ORDER)} (硬件待开始后初始化)")
    else:
        print("[control] 使用 Mock 后端 (无硬件模式)")
        backend = MockBackend()
    app = create_app(backend)
    return app, backend, car


# =========================================================================
# 拉起 Qt HRI 前端（可选，--hri --hri-kiosk --hri-windowed）
# =========================================================================
def _locate_hri_bin(project_root: Path) -> Optional[Path]:
    candidates = [
        project_root / "hri" / "build" / "hri_app",
        project_root / "hri" / "cmake-build-release" / "hri_app",
        project_root / "hri" / "out" / "build" / "hri_app",
    ]
    for c in candidates:
        if c.is_file() and os.access(c, os.X_OK):
            return c
    return None


def _launch_hri_frontend(
    project_root: Path,
    mode: str,
    host: str,
    port: int,
    hri_mode: str,
) -> Optional[subprocess.Popen]:
    bin_path = _locate_hri_bin(project_root)
    if bin_path is None:
        print(f"[control] --hri 指定但未找到 hri_app，跳过启动。请先在 {project_root}/hri/ 下执行 cmake build。")
        print("[control]   构建示例: cd hri && cmake -S . -B build -G Ninja && cmake --build build -j")
        return None

    env = os.environ.copy()
    env.setdefault("HRI_BACKEND_HOST", host if host != "0.0.0.0" else "127.0.0.1")
    env.setdefault("HRI_BACKEND_PORT", str(port))
    env.setdefault("QT_QUICK_CONTROLS_MOBILE", "1")

    cli: List[str] = [str(bin_path)]
    if hri_mode == "kiosk":
        cli.append("--kiosk")
    elif hri_mode == "fullscreen":
        cli.append("--fullscreen")
    elif hri_mode == "windowed":
        cli.append("--windowed")

    print(f"[control] 拉起 HRI 前端: {' '.join(cli)} (host={env['HRI_BACKEND_HOST']} port={env['HRI_BACKEND_PORT']})")
    try:
        # 前端独立进程组；父进程退出时由 start_hri.sh/systemd 负责 respawn
        proc = subprocess.Popen(
            cli,
            env=env,
            stdout=subprocess.DEVNULL if mode == "real" else None,
            stderr=subprocess.DEVNULL if mode == "real" else None,
            start_new_session=True,
        )
    except Exception as exc:
        print(f"[control] 拉起 HRI 前端失败: {exc}")
        return None

    # 等 0.8s 看是否立刻崩
    time.sleep(0.8)
    if proc.poll() is not None:
        print(f"[control] HRI 前端启动失败 (exit={proc.returncode})，但后端继续运行")
        return None
    print(f"[control] HRI 前端 pid={proc.pid}")
    return proc


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m control.main",
        description="rak-hri 比赛控制台后端 (FastAPI + WebSocket)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="使用 Mock 后端（无硬件，x86 开发机调试 GUI 用）",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="使用真实 Orchestrator 硬件后端（需在 Jetson 上运行）",
    )
    parser.add_argument("--host", default="0.0.0.0", help="HTTP/WS 监听地址 (默认 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8500, help="监听端口 (默认 8500)")
    parser.add_argument(
        "--no-stream",
        action="store_true",
        help="真实模式下禁用 MJPEG 推流 (省 CPU)",
    )
    parser.add_argument(
        "--hri",
        dest="hri",
        action="store_const",
        const="kiosk",
        help="启动后自动拉起 Qt HRI 前端（默认 --hri-kiosk 模式）",
    )
    parser.add_argument(
        "--hri-kiosk",
        dest="hri",
        action="store_const",
        const="kiosk",
        help="拉起 Qt HRI 前端：无边框+全屏+置顶+隐藏光标（占满整个桌面/显示屏）",
    )
    parser.add_argument(
        "--hri-fullscreen",
        dest="hri",
        action="store_const",
        const="fullscreen",
        help="拉起 Qt HRI 前端：全屏无边框（保留默认光标）",
    )
    parser.add_argument(
        "--hri-windowed",
        dest="hri",
        action="store_const",
        const="windowed",
        help="拉起 Qt HRI 前端：1024x600 窗口模式（开发机预览）",
    )
    args = parser.parse_args(argv)

    if not args.mock and not args.real:
        args.mock = True
        print("[control] 未指定模式，默认启用 --mock")

    mode = "real" if args.real else "mock"
    car = None
    backend = None
    hri_proc: Optional[subprocess.Popen] = None
    project_root = Path(__file__).resolve().parent.parent

    try:
        app, backend, car = _build_app(
            mode=mode,
            stream=not args.no_stream,
            host=args.host,
            port=args.port,
        )
        print(f"[control] 监听 {args.host}:{args.port}  HTTP+WS")
        print(f"[control] 模式: {mode}   连接 HRI GUI 到 http://{args.host}:{args.port}")

        if args.hri:
            hri_proc = _launch_hri_frontend(
                project_root=project_root,
                mode=mode,
                host=args.host,
                port=args.port,
                hri_mode=args.hri,
            )

        uvicorn.run(app, host=args.host, port=args.port, log_level="info")
        return 0
    except KeyboardInterrupt:
        print("\n[control] 用户中断")
        return 130
    except Exception as exc:
        print(f"[control] 启动失败: {exc}", file=sys.stderr)
        if mode == "real" and "real" in str(exc.__class__).lower() or True:
            print("[control] 提示: 真实模式需要硬件和 SDK；若在开发机请使用 --mock")
        return 1
    finally:
        if hri_proc is not None and hri_proc.poll() is None:
            # 不直接 SIGKILL；systemd 或 start_hri.sh 负责重启；这里仅礼貌 term
            try:
                os.killpg(os.getpgid(hri_proc.pid), signal.SIGTERM)
            except Exception:
                try:
                    hri_proc.terminate()
                except Exception:
                    pass
            try:
                hri_proc.wait(timeout=2.0)
            except Exception:
                try:
                    hri_proc.kill()
                except Exception:
                    pass
        if backend is not None:
            try:
                backend.close()
            except Exception:
                pass
        if car is not None:
            try:
                car.stop()
                car.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
