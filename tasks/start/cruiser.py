# -*- coding: utf-8 -*-
"""巡线 + 触发判定: 封装 cruise_to_trigger.

和 Orchestrator 的关系:
    Orchestrator 提供按键队列 (_pop_key / _flush_keys 等) 与 skip/emergency 状态.
    Cruiser 把这些能力通过构造参数注入, 专注巡线+触发, 不关心按键采集实现.
"""
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from smartcar.whalesbot.tools import logger

from .trigger_configs import TriggerConfigManager
from .triggers import OdometerTrigger, VisionTrigger


# =========================================================================
# Cruiser 需要 Orchestrator 提供的一组按键/状态接口
# =========================================================================
class CruiserHost:
    """Orchestrator 需要实现的最小接口; 方便单元测试时 mock."""

    car: object
    KEY_SKIP: int = 1
    KEY_EMERGENCY: int = 3

    def _pop_key(self) -> int: ...
    def _flush_keys(self) -> None: ...

    @property
    def _skipped(self) -> bool: ...
    @_skipped.setter
    def _skipped(self, v: bool) -> None: ...

    @property
    def _emergency(self) -> bool: ...
    @_emergency.setter
    def _emergency(self, v: bool) -> None: ...


# =========================================================================
# 主封装
# =========================================================================
class Cruiser:
    """把 "从当前位置开始 lane_base 巡线, 满足触发条件就停" 包成一个方法."""

    def __init__(self, host: CruiserHost, cfg_manager: TriggerConfigManager) -> None:
        self.host = host
        self.cfg_manager = cfg_manager
        self.last_result: Dict = {}

    # ------------------------------------------------------------------
    # 对外主入口
    # ------------------------------------------------------------------
    def cruise_to_trigger(self, task_name: str, **cfg_override) -> Dict:
        """按 task_name 解析 cfg, 巡线直到触发/兜底/跳过/急停.

        返回 dict 字段:
            ok / reason / matched_label / traveled / duration
            / start_dist / start_odom
        """
        cfg = self.cfg_manager.resolve(task_name, cfg_override)
        car = self.host.car

        start_distance = float(car.get_distance())
        start_odom = list(car.get_odometry())
        start_time = time.time()

        # 结果占位
        result: Dict = {
            "ok": False,
            "reason": "error",
            "matched_label": None,
            "traveled": 0.0,
            "duration": 0.0,
            "start_dist": start_distance,
            "start_odom": start_odom,
            "task_name": task_name,
        }

        # 创建触发对象 (按 type)
        if cfg["type"] == "vision":
            trigger = VisionTrigger(cfg)
            odometer_trigger: Optional[OdometerTrigger] = None
        else:
            trigger = None
            odometer_trigger = OdometerTrigger(start_distance, cfg["distance"])

        # segments 多段支持 (odometer 专用): 按段累积 distance, 到段边界切换 lane/v_forward
        segments: Optional[List[Dict]] = cfg.get("segments")
        if segments is not None and not isinstance(segments, list):
            segments = None
        if segments is not None and len(segments) == 0:
            segments = None
        seg_idx: int = 0
        seg_cumul: List[float] = []  # 每段结束时的累计里程(相对 start_distance)
        if segments is not None:
            cum = 0.0
            for s in segments:
                cum += float(s.get("distance", 0.0))
                seg_cumul.append(cum)

        # 清理按键 / 状态
        self.host._flush_keys()
        self.host._skipped = False
        self.host._emergency = False

        # 前进速度: 取 lane.v_forward (每任务独立), 缺省回落公共默认 cfg.speed
        # 多段时: 初始取 segments[0].lane.v_forward, 段切换时重设 seg
        def _current_seg() -> Optional[Dict]:
            if segments is None:
                return None
            return segments[min(seg_idx, len(segments) - 1)]

        init_seg = _current_seg()
        seg = (init_seg.get("lane") if init_seg else None) or (cfg.get("lane") or {})
        forward_speed = float(seg.get("v_forward", cfg["speed"]))
        advance = float(cfg.get("advance", 0.0))
        # advance 状态: 触发确认时记录里程计/原因, 继续前进 advance 米再停
        adv_odom: Optional[float] = None  # 触发确认时的里程计
        adv_reason: str = ""
        adv_label: Optional[str] = None

        def _maybe_switch_segment(cur_rel: float) -> None:
            """根据当前累计里程 cur_rel(相对 start_distance), 走到段边界时切 seg 参数.

            段切换: 调用 lane_restore_params 还原上一段 -> lane_apply_params 应用新段
            -> 同步更新 forward_speed.
            """
            nonlocal seg_idx, seg, forward_speed
            if segments is None or seg_cumul is None:
                return
            # 找到 cur_rel 所在的段
            new_idx = 0
            for i, bound in enumerate(seg_cumul):
                if cur_rel >= bound:
                    new_idx = i + 1
                else:
                    break
            if new_idx >= len(segments):
                new_idx = len(segments) - 1
            if new_idx != seg_idx:
                logger.info(
                    f"[cruise:{task_name}] 切段: {seg_idx} -> {new_idx} "
                    f"rel={cur_rel:.3f}m"
                )
                seg_idx = new_idx
                new_seg = segments[seg_idx]
                new_lane = new_seg.get("lane") or {}
                try:
                    car.lane_restore_params()
                except Exception as e:
                    logger.warning(
                        f"[cruise:{task_name}] 切段 lane_restore 异常: {e}"
                    )
                try:
                    car.lane_apply_params(new_lane)
                except Exception as e:
                    logger.warning(
                        f"[cruise:{task_name}] 切段 lane_apply 异常: {e}"
                    )
                seg = new_lane
                forward_speed = float(seg.get("v_forward", cfg["speed"]))
                # 同步更新 lane_base 每 tick 读取的巡线速度, 让 v_forward 段切换立即生效
                try:
                    car._lane_speed = forward_speed
                except Exception:
                    pass

        # 停止判定: 每 tick 都跑 (lane_base 的 end_fuction 会高频调用)
        def _should_stop() -> Tuple[bool, str, Optional[str]]:
            nonlocal adv_odom, adv_reason, adv_label
            # 1) 急停 (用户按 3 或 MyCar._stop_flag 被置位)
            if getattr(car, "_stop_flag", False) or self.host._emergency:
                return True, "emergency", None
            # 2) 跳过
            if self.host._skipped:
                return True, "skip", None
            # 3) start_dist 窗口: 没走到就不检查触发
            cur_dist = float(car.get_distance())
            rel = cur_dist - start_distance
            # 3.1) 多段参数切换 (在 start_dist 窗口内/外都允许切, 以免段边界在 start_dist 内漏切)
            _maybe_switch_segment(rel)
            if rel >= float(cfg["start_dist"]):
                if trigger is not None:  # 视觉
                    trigger.check(car)
                    if trigger.confirmed() and adv_odom is None:
                        adv_odom = cur_dist
                        adv_reason = "vision"
                        adv_label = trigger.confirmed_label()
                elif odometer_trigger is not None:  # 里程计
                    if odometer_trigger.check(cur_dist) and adv_odom is None:
                        adv_odom = cur_dist
                        adv_reason = "odometer"
                        adv_label = None
                # 触发已确认: advance>0 则继续前进到 offset 达标才停; =0 立即停
                if adv_odom is not None:
                    if advance <= 0 or cur_dist - adv_odom >= advance:
                        return True, adv_reason, adv_label
            # 4) 兜底: 最大行驶距离
            if float(cfg["max_run"]) > 0 and rel >= float(cfg["max_run"]):
                logger.warning(
                    f"[cruise:{task_name}] 触发兜底: 已行驶 {rel:.3f}m >= max_run"
                )
                return True, "max_run", None
            # 5) 兜底: 超时
            if float(cfg["time_out"]) > 0:
                if time.time() - start_time > float(cfg["time_out"]):
                    logger.warning(
                        f"[cruise:{task_name}] 触发兜底: 用时 {time.time()-start_time:.1f}s 超时"
                    )
                    return True, "time_out", None
            return False, "", None

        # 巡航期间轻量按键 watcher: 只处理按键队列, 不占串口
        cruise_stop = threading.Event()
        key_thread = threading.Thread(
            target=self._key_watcher_loop,
            args=(cruise_stop, task_name, car),
            daemon=True,
        )
        key_thread.start()

        # 应用路段特调参数 -> 巡线 -> finally 还原(异常/急停也还原)
        try:
            car.lane_apply_params(seg)
        except Exception as e:
            logger.warning(f"[cruise:{task_name}] lane_apply_params 异常: {e}")

        try:
            self._drive_forward(cfg, forward_speed, _should_stop)
        finally:
            cruise_stop.set()
            try:
                key_thread.join(timeout=0.5)
            except Exception:
                pass
            try:
                car.lane_restore_params()
            except Exception:
                pass

        # 最终再判一次, 把 reason / label 同步出来
        _ok, reason, label = _should_stop()
        if not reason:
            reason = "max_run"
        if cfg.get("use_stop", True):
            car.stop()

        result["ok"] = reason in {"vision", "odometer"}
        result["reason"] = reason
        result["matched_label"] = label
        result["traveled"] = float(car.get_distance()) - start_distance
        result["duration"] = time.time() - start_time
        self.last_result = result
        logger.info(
            f"cruise_to_trigger[{task_name}] ok={result['ok']} reason={reason} "
            f"label={label} traveled={result['traveled']:.3f}m dur={result['duration']:.1f}s"
        )
        return result

    # ------------------------------------------------------------------
    # 内部实现
    # ------------------------------------------------------------------
    def _drive_forward(
        self,
        cfg: Dict,
        forward_speed: float,
        stop_check: Callable[[], Tuple[bool, str, Optional[str]]],
    ) -> None:
        """巡线前进: 优先用 lane_base 闭环; 异常退化为开环.

        forward_speed: 本次巡线的权威前进速度 (取路段 lane.v_forward, 缺省 cfg.speed)。
        """
        speed = float(forward_speed)
        use_stop = bool(cfg.get("use_stop", True))
        car = self.host.car
        try:
            car.lane_base(
                speed,
                end_fuction=lambda: stop_check()[0],
                stop=use_stop,
            )
        except Exception as e:
            logger.warning(f"lane_base 异常, 退化为开环前进: {e}")
            car.move_base(
                [speed, 0.0, 0.0],
                end_fuction=lambda: stop_check()[0],
                stop=use_stop,
            )

    def _key_watcher_loop(
        self, cruise_stop: threading.Event, task_name: str, car
    ) -> None:
        """巡航期间消费按键队列: 1=跳过 / 3=急停."""
        while not cruise_stop.is_set():
            key = self.host._pop_key()
            if key == self.host.KEY_SKIP:
                print(f"=== 巡航跳过: {task_name} ===")
                self.host._skipped = True
                try:
                    car.stop()
                except Exception:
                    pass
                return
            if key == self.host.KEY_EMERGENCY:
                print(f"=== 巡航急停: {task_name} (回等待态后重新初始化) ===")
                self.host._emergency = True
                # 置位"待重新初始化"标志: 回等待态时补做臂复位/里程计清零
                # (run.py 的 _CompetitionOrchestrator.wait_start 消费)
                try:
                    self.host._reinit_pending = True
                except Exception:
                    pass
                try:
                    car._stop_flag = True
                except Exception:
                    pass
                try:
                    car.stop()
                except Exception:
                    pass
                return
            time.sleep(0.02)
