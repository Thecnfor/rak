# -*- coding: utf-8 -*-
"""钩子: before_task / after_task 的默认实现 + 调度器 HookManager.

保持简单:
- 一个 HookManager = {before: {task_name: fn, "__all__": fn}, after: 同}
- 调用 before: 先 "__all__", 再任务专属
- 调用 after:  只调任务专属 (如果以后要通配, 再加 "__all__" 也很简单)
- 钩子异常永远只 warning, 不中断主流程
"""
from typing import Callable, Dict, Optional

from smartcar.whalesbot.tools import logger


# =========================================================================
# 默认钩子
# =========================================================================
def default_before_any(car, task_name: str) -> None:
    """所有任务触发后、执行 run() 前的统一提示: beep + 日志."""
    logger.info(f"触发任务点: {task_name}")
    try:
        car.beep()
    except Exception:
        pass


def default_after_sorting(car, task_name: str) -> None:
    """sorting 结束清零里程计 & 累计路程 (ordering 的里程计触发以此为起点)."""
    logger.info("sorting 结束 → 清零里程计与累计距离")
    car.reset_position()
    car.get_odometry(True)
    car.get_distance(True)


# =========================================================================
# 调度器
# =========================================================================
class HookManager:
    """before / after 钩子的管理与调度."""

    def __init__(self) -> None:
        self.before: Dict[str, Callable] = {
            "__all__": default_before_any,
        }
        self.after: Dict[str, Callable] = {
            "sorting": default_after_sorting,
        }

    # ---- 配置 API ----

    def set_before(self, task_name: str, hook: Optional[Callable]) -> None:
        """task_name 可为 "__all__"; hook=None 表示删除."""
        if hook is None:
            self.before.pop(task_name, None)
        else:
            self.before[task_name] = hook

    def set_after(self, task_name: str, hook: Optional[Callable]) -> None:
        if hook is None:
            self.after.pop(task_name, None)
        else:
            self.after[task_name] = hook

    def clear_all(self) -> None:
        self.before.clear()
        self.after.clear()

    # ---- 调用 API ----

    def call_before(self, car, task_name: str) -> None:
        """先调 "__all__" (如果有), 再调任务专属 (如果有)."""
        funcs = []
        if "__all__" in self.before:
            funcs.append(self.before["__all__"])
        if task_name in self.before and task_name != "__all__":
            funcs.append(self.before[task_name])
        for fn in funcs:
            try:
                fn(car, task_name)
            except Exception as e:
                logger.warning(f"before_hook[{task_name}] 异常: {e}")

    def call_after(self, car, task_name: str) -> None:
        if task_name in self.after:
            try:
                self.after[task_name](car, task_name)
            except Exception as e:
                logger.warning(f"after_hook[{task_name}] 异常: {e}")
