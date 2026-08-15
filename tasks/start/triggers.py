# -*- coding: utf-8 -*-
"""触发判定: 视觉触发 / 里程计触发, 各自独立成类.

和 Orchestrator 解耦: 这两个类只看 car / cfg / 当前状态,
不负责巡线 / 按键 / 多线程, 外部 (Cruiser) 来按 tick 调用.
"""
from dataclasses import dataclass
from typing import Dict, List, Optional

from smartcar import CountRecord
from smartcar.whalesbot.tools import logger


# =========================================================================
# 里程计触发
# =========================================================================
class OdometerTrigger:
    """按"累计路程相对起点 >= distance"判定是否命中."""

    def __init__(self, start_distance: float, distance: float) -> None:
        self.start_distance = float(start_distance)
        self.distance = float(distance)

    def check(self, current_distance: float) -> bool:
        rel = float(current_distance) - self.start_distance
        return rel >= self.distance

    def traveled(self, current_distance: float) -> float:
        return float(current_distance) - self.start_distance


# =========================================================================
# 视觉触发
# =========================================================================
@dataclass
class VisionHit:
    """一帧命中的细节: label + 首个匹配的 det(完整 8 字段)."""

    label: str
    det: list


class VisionTrigger:
    """侧视实时检测命中 labels 集合 + 连续 confirm 帧确认.

    用法:
        vt = VisionTrigger(cfg)
        while driving:
            hit = vt.check(car)
            if vt.confirmed():
                break
    """

    def __init__(self, cfg: Dict) -> None:
        self.labels = set(cfg["labels"])
        self.min_score = float(cfg["min_score"])
        self.confirm = int(cfg["confirm"])
        self.fresh = bool(cfg["fresh"])
        self.max_age = float(cfg.get("max_age", 0.3))

        # CountRecord(N): 连续 N 次 True 才返回 True
        self._confirm = CountRecord(self.confirm)
        self._last_hit: Optional[VisionHit] = None
        self._last_frame_dets: List = []

    # ----- 对外接口 -----

    def check(self, car) -> Optional[VisionHit]:
        """对当前帧做一次检查; 返回本帧命中的 VisionHit(可能 None)."""
        dets = self._read_dets(car)
        self._last_frame_dets = dets
        hit = self._pick_first_hit(dets)
        self._last_hit = hit
        self._confirm(hit is not None)
        return hit

    def confirmed(self) -> bool:
        """是否已达到连续 confirm 帧命中."""
        return bool(self._confirm(True)) and self._last_hit is not None

    def confirmed_label(self) -> Optional[str]:
        return self._last_hit.label if self.confirmed() else None

    def last_hit(self) -> Optional[VisionHit]:
        return self._last_hit

    def last_frame_dets(self) -> List:
        return list(self._last_frame_dets)

    # ----- 内部 -----

    def _read_dets(self, car) -> List:
        try:
            if self.fresh:
                return car.get_realtime_detections(fresh=True)
            return car.get_realtime_detections(fresh=False, max_age=self.max_age)
        except Exception as e:
            logger.warning(f"VisionTrigger 取检测结果异常: {e}")
            return []

    def _pick_first_hit(self, dets: List) -> Optional[VisionHit]:
        for det in dets:
            # 格式: [cls_id, obj_id, label, score, x_c, y_c, w, h]
            try:
                label, score = str(det[2]), float(det[3])
            except Exception:
                continue
            if label in self.labels and score >= self.min_score:
                return VisionHit(label=label, det=list(det))
        return None
