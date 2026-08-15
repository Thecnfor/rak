# -*- coding: utf-8 -*-
"""触发配置: 8 个任务的触发条件静态表 + 运行时覆盖管理。

一个任务的触发 = 走静态表 + 运行时 override 合并, 统一 resolve_config 拿到。
"""
from copy import deepcopy
from typing import Dict, Optional


TASK_ORDER = [
    "seeding",
    "target_detection",
    "watering",
    "shooting",
    "harvesting",
    "sorting",
    "ordering",
    "delivery",
]


# =========================================================================
# 每个任务的默认触发条件
# =========================================================================
# 触发类型两种:
#   "odometer": 按累计行驶距离触发 (distance 米, 可调)
#   "vision":   用侧视实时检测命中 label 集合触发
#
# 公共参数 (两种都有):
#   speed:       巡线速度 m/s
#   max_run:     兜底最大行驶距离米; 视觉触发必填, 防止漏检过站
#   time_out:    兜底超时秒, 0 不启用
#   start_dist:  触发前先行驶多少米才开始检查 (0 立即开始)
#   use_stop:    命中/兜底后是否调用 car.stop, 默认 True
#
# vision 额外参数:
#   labels:      命中即可触发的 label 集合 (任一命中)
#   min_score:   单帧最低置信度 0~1
#   confirm:     连续 N 帧命中才算确认 (抑制单帧误检)
#   fresh:       True=每帧同步推理(最准稍慢); False=用实时缓存(最快)
#   max_age:     fresh=False 时允许的实时缓存最大年龄秒
#
# odometer 额外参数:
#   distance:    相对 start 的累计路程阈值米, >= 就命中
# =========================================================================
TASK_TRIGGER: Dict[str, Dict] = {
    # ---------------- 里程计触发 ----------------
    "seeding": {
        "type": "odometer",
        "distance": 0.85,
        "speed": 0.3,
        "max_run": 1.5,
        "time_out": 0.0,
        "start_dist": 0.0,
    },
    "ordering": {
        "type": "odometer",
        "distance": 1.5,
        "speed": 0.3,
        "max_run": 2.0,
        "time_out": 0.0,
        "start_dist": 0.0,
    },
    # ---------------- 视觉触发 ----------------
    "target_detection": {
        "type": "vision",
        "labels": ["animal"],
        "min_score": 0.6,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "speed": 0.3,
        "max_run": 2.5,
        "time_out": 20.0,
        "start_dist": 0.0,
    },
    "watering": {
        "type": "vision",
        "labels": ["water", "water_l1", "water_l2", "water_l3"],
        "min_score": 0.6,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "speed": 0.3,
        "max_run": 3.0,
        "time_out": 30.0,
        "start_dist": 0.0,
    },
    "shooting": {
        "type": "vision",
        "labels": ["animal"],
        "min_score": 0.6,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "speed": 0.3,
        "max_run": 3.8,
        "time_out": 30.0,
        "start_dist": 0.0,
    },
    "harvesting": {
        "type": "vision",
        "labels": ["ball_yellow", "ball_blue"],
        "min_score": 0.55,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "speed": 0.3,
        "max_run": 3.0,
        "time_out": 30.0,
        "start_dist": 0.0,
    },
    "sorting": {
        "type": "vision",
        "labels": [
            "ball_yellow",
            "ball_blue",
            "label_yellow",
            "label_blue",
        ],
        "min_score": 0.55,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "speed": 0.3,
        "max_run": 2.5,
        "time_out": 25.0,
        "start_dist": 0.0,
    },
    "delivery": {
        "type": "vision",
        "labels": ["name"],
        "min_score": 0.5,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "speed": 0.3,
        "max_run": 3.8,
        "time_out": 30.0,
        "start_dist": 0.0,
    },
}


# 一些合理的默认值, 用于 resolve 时 setdefault 补全字段
_COMMON_DEFAULTS = {
    "speed": 0.3,
    "max_run": 0.0,
    "time_out": 0.0,
    "start_dist": 0.0,
    "use_stop": True,
}
_VISION_DEFAULTS = {
    "min_score": 0.5,
    "confirm": 3,
    "fresh": False,
    "max_age": 0.3,
}
_ODOMETER_DEFAULTS: Dict = {}


def default_trigger_config(task_name: str) -> Dict:
    """返回一个任务的默认触发配置深拷贝 (外部改它不会污染静态表)."""
    if task_name not in TASK_TRIGGER:
        raise ValueError(f"未知任务名 {task_name}; 可选: {sorted(TASK_TRIGGER.keys())}")
    return deepcopy(TASK_TRIGGER[task_name])


def resolve_config(base_cfg: Dict, override: Optional[Dict] = None) -> Dict:
    """合并基础配置 + 运行时覆盖 + 默认补全字段."""
    cfg = deepcopy(base_cfg)
    if override:
        cfg.update(override)

    # 公共参数补全
    for k, v in _COMMON_DEFAULTS.items():
        cfg.setdefault(k, v)

    if cfg.get("type") == "vision":
        for k, v in _VISION_DEFAULTS.items():
            cfg.setdefault(k, v)
    elif cfg.get("type") == "odometer":
        for k, v in _ODOMETER_DEFAULTS.items():
            cfg.setdefault(k, v)
    else:
        raise ValueError(
            f"未知触发类型 {cfg.get('type')!r}, 可选 'vision' / 'odometer'"
        )
    return cfg


class TriggerConfigManager:
    """运行时的触发配置覆盖管理器 (替代之前 Orchestrator 里的 _trigger_overrides)."""

    def __init__(self) -> None:
        self._overrides: Dict[str, Dict] = {}

    def override(self, task_name: str, **kwargs) -> None:
        """为任务 task_name 增加/更新运行时覆盖 (会合并到默认表)."""
        if task_name not in TASK_TRIGGER:
            raise ValueError(
                f"未知任务名 {task_name}; 可选: {sorted(TASK_TRIGGER.keys())}"
            )
        cur = self._overrides.setdefault(task_name, {})
        cur.update(kwargs)

    def clear_override(self, task_name: Optional[str] = None) -> None:
        """清掉某个任务或所有任务的运行时覆盖."""
        if task_name is None:
            self._overrides.clear()
        else:
            self._overrides.pop(task_name, None)

    def resolve(self, task_name: str, extra_override: Optional[Dict] = None) -> Dict:
        """返回最终生效的 config (默认表 + 本 manager 的覆盖 + 本次一次性 extra_override)."""
        base = default_trigger_config(task_name)
        if task_name in self._overrides:
            base.update(self._overrides[task_name])
        return resolve_config(base, extra_override)
