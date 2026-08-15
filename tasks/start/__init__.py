# -*- coding: utf-8 -*-
"""tasks.start: 比赛模式任务编排子模块（从 orchestrator 拆分）。

子模块职责:
    trigger_configs  → 静态触发配置 + 解析/覆盖
    triggers         → 视觉 / 里程计触发判定
    cruiser          → 巡线 + 触发判定 + 急停/跳过兜底
    hooks            → before / after 钩子默认实现 + 调度器
    runner           → run_next_task / run_all / run_task_module
"""
from .trigger_configs import (
    TASK_ORDER,
    TASK_TRIGGER,
    default_trigger_config,
    TriggerConfigManager,
)
from .triggers import OdometerTrigger, VisionTrigger
from .cruiser import Cruiser
from .hooks import HookManager, default_before_any, default_after_sorting
from .runner import TaskRunner


__all__ = [
    "TASK_ORDER",
    "TASK_TRIGGER",
    "default_trigger_config",
    "TriggerConfigManager",
    "OdometerTrigger",
    "VisionTrigger",
    "Cruiser",
    "HookManager",
    "default_before_any",
    "default_after_sorting",
    "TaskRunner",
]
