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
#   speed:       巡线速度 m/s (统一默认 0.3, 见 _COMMON_DEFAULTS);
#                单个任务要改速度在 lane.v_forward 里配(优先级更高)
#   max_run:     兜底最大行驶距离米; 视觉触发必填, 防止漏检过站
#   time_out:    兜底超时秒, 0 不启用
#   start_dist:  触发前先行驶多少米才开始检查 (0 立即开始)
#   use_stop:    命中/兜底后是否调用 car.stop, 默认 True
#   advance:     触发确认后继续巡线前进 N 米再停车 (视觉/里程计通用);
#                0 表示触发即停。用于"识别到目标后多走一段"再停到对准点
#   end_pose:    任务 run() 结束后编排器自动 go_to_pose 到的绝对位姿
#                [x, y, theta](当前里程计坐标系, 弧度); None=不钉姿势。
#                方便下一个任务从已知姿势开始、视觉发现对齐
#   lane:        路段巡线特调子配置 dict (一个任务巡航 = 一个路段):
#                kp/ki/kd/limits -> 转向 PID (lane_pid_angle) —— 每个任务独立调
#                deadzone        -> da 进 PID 前死区
#                corr_threshold/corr_weight -> correction 居中通道
#                ema/lane_timeout -> 前视推理滤波
#                v_forward       -> 前进速度 (m/s, 恒速; 优先级高于公共 speed)
#                缺省项回落到 tasks/tools/cfg.py 全局默认; 每段路独立调参,
#                巡线跑完自动还原, 不污染其他路段
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
    # ================= 里程计触发 =================
    "seeding": {
        "type": "odometer",  # 触发类型: odometer=按行驶距离触发 / vision=视觉识别触发
        "distance": 0.87,  # 累计行驶距离(米) >= 该值即触发 (仅 odometer 用)
        "max_run": 1.5,  # 兜底: 最多再行驶这么远(米), 防止触发条件永远不满足而一直开
        "time_out": 0.0,  # 兜底: 超时(秒)强制停, 0=不启用
        "start_dist": 0.0,  # 先行驶多少米后才开始检查触发条件 (0=一开始就检查)
        "lane": {  # 本路段巡线特调(每任务独立, 跑完自动还原)
            "kp": 1,  # 转向 PID Kp: 弯道转不过来加大 / 直道蛇形减小
            "kd": 0.0,  # 转向 PID Kd(阻尼): 摆动大加大 / 转向迟钝减小
            "deadzone": 0.0,  # da 进 PID 前死区: 直线仍抖加大
            "v_forward": 0.50,  # 恒速前进速度(m/s), 不填则回落公共默认 speed 0.3
        },
    },
    "ordering": {
        "type": "odometer",
        "distance": 1.5,  # 行驶 1.5m 触发 (sorting 结束已清零里程计, 从 0 起算)
        "max_run": 2.0,
        "time_out": 0.0,
        "start_dist": 0.0,
        "lane": {
            "kp": 1.5,
            "kd": 0.0,
            "deadzone": 0.0,
            "v_forward": 0.7,
        },
    },
    # ================= 视觉触发 =================
    # 公共参数字段见 odometer 块; 视觉额外字段:
    #   labels: 命中任一 label 即可触发 (与 task 模型 labels.txt 对应)
    #   min_score: 单帧最低置信度 0~1, 低于则不算命中
    #   confirm: 连续 N 帧命中才算确认 (抑制单帧误检)
    #   fresh: True=每帧同步推理(最准稍慢); False=用实时缓存(最快)
    #   max_age: fresh=False 时允许的实时缓存最大年龄(秒)
    "target_detection": {
        "type": "vision",
        "labels": ["animal"],  # 识别到 animal 即触发
        "min_score": 0.6,  # 置信度 >=0.6 才算命中
        "confirm": 3,  # 连续 3 帧命中才确认
        "fresh": False,  # 用实时缓存, 最快
        "max_age": 0.3,  # 缓存最多 0.3s 内的检测结果
        "max_run": 20.5,  # 视觉兜底必填: 2.5m 内没识别到也停, 防过站
        "time_out": 200.0,  # 20s 没触发强制停
        "start_dist": 0.0,
        "lane": {  # 本路段巡线特调(每任务独立, 跑完自动还原)
            "kp": 2,  # 转向 PID Kp: 弯道转不过来加大 / 直道蛇形减小
            "kd": 0.0,  # 转向 PID Kd(阻尼): 摆动大加大 / 转向迟钝减小
            "deadzone": 0.0,  # da 进 PID 前死区: 直线仍抖加大
            "v_forward": 0.4,  # 恒速前进速度(m/s), 不填则回落公共默认 speed 0.3
        },
    },
    "watering": {
        "type": "vision",
        "labels": [
            # "water",
            "water_l1",
            "water_l2",
            "water_l3",
        ],  # 识别到任一水塔等级即触发
        "min_score": 0.4,
        "confirm": 1,
        "fresh": False,
        "max_age": 0.3,
        "max_run": 300.0,
        "time_out": 300.0,
        "start_dist": 0.0,
        "lane": {  # 本路段巡线特调(每任务独立, 跑完自动还原)
            "kp": 2,
            "kd": 0.0,
            "deadzone": 0.0,
            "v_forward": 0.5,
        },
    },
    "shooting": {
        "type": "vision",
        "labels": ["animal"],  # 识别到动物即触发
        "min_score": 0.7,
        "confirm": 10,
        "fresh": False,
        "max_age": 0.3,
        "max_run": 500.0,
        "time_out": 300.0,
        "start_dist": 1.5,
        "lane": {  # 本路段巡线特调(每任务独立, 跑完自动还原)
            "kp": 2,
            "kd": 0.0,
            "deadzone": 0.0,
            "v_forward": 0.6,
        },
    },
    "harvesting": {
        "type": "vision",
        "labels": ["ball_yellow", "ball_blue"],  # 识别到黄/蓝球即触发
        "min_score": 0.55,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "max_run": 5.0,
        "time_out": 300.0,
        "start_dist": 0.0,
        "lane": {  # 本路段巡线特调(每任务独立, 跑完自动还原)
            "kp": 3.4,
            "kd": 0.0,
            "deadzone": 0.0,
            "v_forward": 0.5,
        },
    },
    "sorting": {
        "type": "vision",
        "labels": [
            "label_yellow",
            "label_blue",  # 识别到黄/蓝标签即触发
        ],
        "min_score": 0.55,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "max_run": 2.5,
        "time_out": 250.0,
        "start_dist": 0.0,
        "lane": {  # 本路段巡线特调(每任务独立, 跑完自动还原)
            "kp": 3.5,
            "kd": 0.0,
            "deadzone": 0.0,
            "v_forward": 0.6,
        },
    },
    "delivery": {
        "type": "vision",
        "labels": ["name"],  # 识别到名字牌即触发
        "min_score": 0.5,
        "confirm": 3,
        "fresh": False,
        "max_age": 0.3,
        "max_run": 1.8,
        "time_out": 300.0,
        "start_dist": 0.0,
        "lane": {  # 本路段巡线特调(每任务独立, 跑完自动还原)
            "kp": 5.0,
            "kd": 0.0,
            "deadzone": 0.0,
            "v_forward": 0.6,
        },
    },
}


# 一些合理的默认值, 用于 resolve 时 setdefault 补全字段
_COMMON_DEFAULTS = {
    "speed": 0.3,
    "max_run": 0.0,
    "time_out": 0.0,
    "start_dist": 0.0,
    "use_stop": True,
    "advance": 0.0,
    "end_pose": None,
    "lane": {},
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
