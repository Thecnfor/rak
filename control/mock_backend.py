from __future__ import annotations

import random
import threading
import time
from typing import Dict, List, Optional

from .backend import CarBackend, EventType, TASK_ORDER


class MockBackend(CarBackend):
    """模拟后端：不依赖硬件/Paddle，x86 即可跑。

    后台线程按 TASK_ORDER 逐任务推进：task:started → 短延时 → task:done。
    支持 from_index 跳过前置任务、stop/skip/reset、setTaskSpeed、按键注入。
    """

    TASK_DURATION_BASE = 2.5

    def __init__(self) -> None:
        super().__init__()
        self._bg_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._skip_event = threading.Event()
        self._cmd_lock = threading.Lock()
        self._odom_dist = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start_bg(self) -> None:
        if self._odom_thread and self._odom_thread.is_alive():
            return
        self._stop_event.clear()
        self._odom_thread = threading.Thread(target=self._odom_loop, daemon=True)
        self._odom_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        self._skip_event.set()
        if self._bg_thread:
            self._bg_thread.join(timeout=2.0)
        if self._odom_thread:
            self._odom_thread.join(timeout=1.0)

    # ------------------------------------------------------------------
    # 命令
    # ------------------------------------------------------------------
    def start(self, from_index: int = -1) -> None:
        with self._cmd_lock:
            if self._active:
                raise RuntimeError("已有任务运行中")
            self._skip_event.clear()
            self._active = True
            start_idx = max(0, int(from_index)) if from_index >= 0 else 0
            for i, name in enumerate(TASK_ORDER):
                self._set_task_status(
                    name, "done" if i < start_idx else "pending"
                )
            self._emit({"type": EventType.RUN_STARTED})
            self._bg_thread = threading.Thread(
                target=self._run_all_worker, args=(start_idx,), daemon=True
            )
            self._bg_thread.start()

    def run_task(self, name: str) -> None:
        if name not in TASK_ORDER:
            raise ValueError(f"未知任务: {name}")
        with self._cmd_lock:
            if self._active:
                raise RuntimeError("已有任务运行中")
            self._skip_event.clear()
            self._active = True
            self._bg_thread = threading.Thread(
                target=self._run_single_worker, args=(name,), daemon=True
            )
            self._bg_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._skip_event.set()

    def skip(self) -> None:
        self._skip_event.set()

    def reset(self) -> None:
        self.stop()
        time.sleep(0.1)
        with self._cmd_lock:
            for name in TASK_ORDER:
                self._set_task_status(name, "pending")
            self._current_task = None
            self._active = False
            self._stop_event.clear()
            self._skip_event.clear()
            self._odom_dist = 0.0
            self._odom_x = 0.0
            self._odom_y = 0.0
        self._emit({"type": EventType.RESET})

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------
    def current_odometry(self) -> Dict[str, float]:
        return {
            "x": round(self._odom_x, 3),
            "y": round(self._odom_y, 3),
            "dist": round(self._odom_dist, 3),
        }

    # ------------------------------------------------------------------
    # Worker 线程
    # ------------------------------------------------------------------
    def _run_all_worker(self, start_idx: int) -> None:
        try:
            for i in range(start_idx, len(TASK_ORDER)):
                if self._stop_event.is_set():
                    break
                name = TASK_ORDER[i]
                self._execute_task(name)
                if self._stop_event.is_set():
                    break
                if self._task_status.get(name) != "done":
                    # skipped 或 failed，不继续推进（保持行为一致）
                    pass
        finally:
            with self._cmd_lock:
                self._active = False
                self._current_task = None
            self._emit(
                {"type": EventType.RUN_FINISHED, "reason": "completed" if not self._stop_event.is_set() else "stopped"}
            )

    def _run_single_worker(self, name: str) -> None:
        try:
            self._execute_task(name)
        finally:
            with self._cmd_lock:
                self._active = False
                self._current_task = None
            self._emit(
                {"type": EventType.RUN_FINISHED, "reason": "single_completed"}
            )

    def _execute_task(self, name: str) -> None:
        self._current_task = name
        self._set_task_status(name, "running")
        self._emit({"type": EventType.TASK_STARTED, "task": name})

        speed = self._task_speeds.get(name, 0.3)
        duration = max(0.4, self.TASK_DURATION_BASE * (0.3 / max(0.05, speed)))
        tick = 0.05
        elapsed = 0.0
        while elapsed < duration:
            if self._stop_event.is_set():
                self._emit(
                    {
                        "type": EventType.TASK_SKIPPED,
                        "task": name,
                        "reason": "stopped",
                    }
                )
                self._set_task_status(name, "pending")
                return
            if self._skip_event.is_set():
                self._skip_event.clear()
                self._emit(
                    {
                        "type": EventType.TASK_SKIPPED,
                        "task": name,
                        "reason": "skip_requested",
                    }
                )
                self._set_task_status(name, "pending")
                return
            time.sleep(tick)
            elapsed += tick

        # 5% 概率模拟失败（方便 UI 观察）
        if random.random() < 0.02:
            self._set_task_status(name, "failed")
            self._emit(
                {
                    "type": EventType.TASK_ERROR,
                    "task": name,
                    "error": f"{name} 模拟失败（重试）",
                }
            )
            return

        self._set_task_status(name, "done")
        self._emit({"type": EventType.TASK_DONE, "task": name})

    # ------------------------------------------------------------------
    # 里程计模拟（推送 odom 事件）
    # ------------------------------------------------------------------
    def _odom_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                if self._active:
                    self._odom_dist += 0.01
                    self._odom_x += 0.01
                    self._odom_y += 0.002
                self._emit(
                    {
                        "type": EventType.ODOM,
                        "x": round(self._odom_x, 3),
                        "y": round(self._odom_y, 3),
                        "dist": round(self._odom_dist, 3),
                    }
                )
            except Exception:
                pass
            time.sleep(0.2)
