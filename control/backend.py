from __future__ import annotations

import abc
import threading
import time
from typing import Any, Callable, Dict, List, Optional


class EventType:
    HELLO = "hello"
    RUN_STARTED = "run:started"
    RUN_FINISHED = "run:finished"
    TASK_STARTED = "task:started"
    TASK_DONE = "task:done"
    TASK_SKIPPED = "task:skipped"
    TASK_ERROR = "task:error"
    RESET = "reset"
    ERROR = "error"
    ODOM = "odom"


TASK_ORDER: List[str] = [
    "seeding",
    "target_detection",
    "watering",
    "shooting",
    "harvesting",
    "sorting",
    "ordering",
    "delivery",
]


TASK_INFO: Dict[str, Dict[str, Any]] = {
    "seeding": {
        "name": "seeding",
        "name_cn": "播种",
        "description": "沿里程计行至播种区完成播种",
        "trigger": "odometer",
    },
    "target_detection": {
        "name": "target_detection",
        "name_cn": "识别虫害",
        "description": "视觉识别虫害目标并确认",
        "trigger": "vision",
    },
    "watering": {
        "name": "watering",
        "name_cn": "灌溉",
        "description": "巡线至水源区完成灌溉",
        "trigger": "vision",
    },
    "shooting": {
        "name": "shooting",
        "name_cn": "射击除害",
        "description": "视觉锁定害虫目标并射击",
        "trigger": "vision",
    },
    "harvesting": {
        "name": "harvesting",
        "name_cn": "作物收集",
        "description": "收集黄色/蓝色作物球",
        "trigger": "vision",
    },
    "sorting": {
        "name": "sorting",
        "name_cn": "作物储存",
        "description": "按颜色归类储存作物",
        "trigger": "vision",
    },
    "ordering": {
        "name": "ordering",
        "name_cn": "订单获取",
        "description": "巡线行至订单点获取订单",
        "trigger": "odometer",
    },
    "delivery": {
        "name": "delivery",
        "name_cn": "订单配送",
        "description": "按订单将作物配送至目标点",
        "trigger": "vision",
    },
}


class CarBackend(abc.ABC):
    """后端抽象基类：统一的事件回调 + 命令接口。"""

    def __init__(self) -> None:
        self._event_lock = threading.Lock()
        self._event_callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self._task_speeds: Dict[str, float] = {t: 0.3 for t in TASK_ORDER}
        self._task_status: Dict[str, str] = {t: "pending" for t in TASK_ORDER}
        self._active = False
        self._current_task: Optional[str] = None

    # ------------------------------------------------------------------
    # 事件订阅
    # ------------------------------------------------------------------
    def add_event_callback(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        with self._event_lock:
            self._event_callbacks.append(cb)

    def remove_event_callback(self, cb: Callable[[Dict[str, Any]], None]) -> None:
        with self._event_lock:
            try:
                self._event_callbacks.remove(cb)
            except ValueError:
                pass

    def _emit(self, event: Dict[str, Any]) -> None:
        with self._event_lock:
            callbacks = list(self._event_callbacks)
        for cb in callbacks:
            try:
                cb(event)
            except Exception as exc:
                print(f"[backend] event cb error: {exc}")

    # ------------------------------------------------------------------
    # 状态快照（用于 /api/hello 与连接后首次推送）
    # ------------------------------------------------------------------
    def hello_snapshot(self) -> Dict[str, Any]:
        tasks = []
        for key in TASK_ORDER:
            info = dict(TASK_INFO[key])
            info["speed"] = self._task_speeds.get(key, 0.3)
            info["status"] = self._task_status.get(key, "pending")
            tasks.append(info)
        return {
            "type": EventType.HELLO,
            "tasks": tasks,
            "active": self._active,
            "current": self._current_task if self._current_task is not None else "",
            "odom": self.current_odometry(),
        }

    # ------------------------------------------------------------------
    # 命令接口（由 FastAPI 层调用）
    # ------------------------------------------------------------------
    @abc.abstractmethod
    def start(self, from_index: int = -1) -> None:
        """一键启动全流程；from_index >= 0 表示从该任务起启动，之前的标记 done。"""

    @abc.abstractmethod
    def run_task(self, name: str) -> None:
        """仅运行单个任务。"""

    @abc.abstractmethod
    def stop(self) -> None:
        """急停。"""

    @abc.abstractmethod
    def skip(self) -> None:
        """跳过当前任务。"""

    @abc.abstractmethod
    def reset(self) -> None:
        """清空 done 集合，重置状态。"""

    def set_task_speed(self, name: str, speed: float) -> None:
        """覆盖指定任务的速度（m/s）。"""
        if name in self._task_speeds:
            self._task_speeds[name] = max(0.05, min(2.0, float(speed)))

    # ------------------------------------------------------------------
    # 状态查询
    # ------------------------------------------------------------------
    def tasks_snapshot(self) -> List[Dict[str, Any]]:
        out = []
        for key in TASK_ORDER:
            info = dict(TASK_INFO[key])
            info["speed"] = self._task_speeds.get(key, 0.3)
            info["status"] = self._task_status.get(key, "pending")
            out.append(info)
        return out

    def status_snapshot(self) -> Dict[str, Any]:
        return {
            "active": self._active,
            "current": self._current_task,
            "tasks": self.tasks_snapshot(),
            "odom": self.current_odometry(),
        }

    def current_odometry(self) -> Dict[str, float]:
        return {"x": 0.0, "y": 0.0, "dist": 0.0}

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start_bg(self) -> None:
        """启动后台线程（里程计轮询等）。默认空实现。"""

    def close(self) -> None:
        """清理资源。默认空实现。"""

    # ------------------------------------------------------------------
    # 内部状态变更帮助
    # ------------------------------------------------------------------
    def _set_task_status(self, name: str, status: str) -> None:
        if name in self._task_status:
            self._task_status[name] = status
