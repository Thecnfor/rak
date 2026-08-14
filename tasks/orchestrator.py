# -*- coding: utf-8 -*-
"""
按键驱动的任务编排器

背景：
    比赛允许多次重试，但每个任务只能计分一次；重试时车要回到起点重头跑，
    期间只能通过车上的按键（Key4Btn）控制，不能改代码。

方案：
    - 使用车内按键（4 键）驱动：
        按键 4 = 一键启动 / 重来（重来时会跳过本次运行中已完成的"继续"）
        按键 1 = 跳过当前任务（不标记完成，下次重来还能补做）
        按键 3 = 急停（复用 MyCar 自带的按键线程急停逻辑）
    - "已完成的任务"只保存在内存中（本次运行内有效）；
      想要彻底重头跑，直接重启程序即可（内存不持久化，天然全新一次）。
    - （新增）从当前位置巡线到"任务触发点"：
        每个任务点的触发条件（视觉/里程计）定义在 TASK_TRIGGER 中；
        巡线过程中持续检测侧视实时检测结果或里程计累计距离，
        命中即停车 → 调用 before_task_hook → （可选）执行任务 run(car)
        → 调用 after_task_hook → 标记完成，继续下一个任务点。
"""
import importlib
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from smartcar.whalesbot.tools import logger


class Orchestrator:
    """任务编排器：负责按键采集、运行状态维护与任务调度。"""

    # 任务在比赛场上的执行顺序（保持与 run.py run_all 一致）
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

    # 各按键的语义
    KEY_START = 4  # 一键启动 / 重来（跳过已完成）
    KEY_SKIP = 1  # 跳过当前任务（本次运行不执行，下次重来仍可补做）
    KEY_EMERGENCY = 3  # 急停（与 MyCar.key_thread_func 保持一致）

    # ------------------------------------------------------------------
    # 任务触发条件（默认配置，可通过 override_trigger() 运行时覆盖）
    #
    # 触发类型两种:
    #   "odometer": 按里程计累计距离触发（distance 米，可调）
    #   "vision":   侧视实时检测命中 label 集合触发
    #
    # 公共参数（两种都有）:
    #   speed:       巡线速度（m/s）
    #   max_run:     兜底最大行驶距离（米），视觉触发必填，防止漏检过站
    #   time_out:    兜底超时（秒），0 表示不启用
    #   start_dist:  触发前先行驶多少米才开始检查（米，0 表示立即开始）
    #   use_stop:    结束后是否立即调用 car.stop，默认 True
    #
    # vision 附加参数:
    #   labels:       命中即可触发的 label 集合（任一命中即可）
    #   min_score:    单帧最低置信度（0~1，低于直接丢弃）
    #   confirm:      连续 N 帧命中才算确认（抑制单帧误检）
    #   fresh:        True=每帧都同步推理(最准,稍慢); False=用实时缓存(最快)
    #   max_age:      当 fresh=False 时，允许使用的实时缓存最大年龄(秒)
    #
    # odometer 附加参数:
    #   distance:     相对 start 的累计行驶距离阈值（米），>= 就触发
    #
    # 顺序对应 TASK_ORDER:
    #   seeding            -> 里程计触发（距离做成可配置）
    #   target_detection   -> 看到 animal 触发
    #   watering           -> 看到 water_* / water 触发
    #   shooting           -> 看到 animal 触发
    #   harvesting         -> 看到 ball（黄/蓝）触发
    #   sorting            -> 看到蓝/黄（ball 或 label）触发
    #   ordering           -> 里程计触发（基于上一个任务清零后）
    #   delivery           -> 看到 name 触发
    # ------------------------------------------------------------------
    TASK_TRIGGER: Dict[str, Dict] = {
        "seeding": {
            "type": "odometer",
            "distance": 0.85,  # 默认值; 可调用 override_trigger 调整
            "speed": 0.3,
            "max_run": 1.5,
            "time_out": 0,
            "start_dist": 0.0,
        },
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
        "ordering": {
            "type": "odometer",
            "distance": 1.5,  # 默认值; 上一个任务(sorting)结束时清零里程计
            "speed": 0.3,
            "max_run": 2.0,
            "time_out": 0,
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

    # ------------------------------------------------------------------
    # 默认的 before/after 钩子
    #
    # 钩子签名: hook(car, task_name) -> None
    # 内置默认钩子:
    #   sorting 的 after_hook: 清零里程计 + 里程计/距离归零
    #       让下一个任务 ordering 的"基于清零后里程计"触发距离生效
    # 调用方也可通过 set_before_hook / set_after_hook 自定义
    # ------------------------------------------------------------------
    @staticmethod
    def _default_after_sorting(car, task_name):
        """sorting 完成后清零里程计, 供 ordering 的里程计触发做起点."""
        logger.info("sorting 结束 → 清零里程计与累计距离")
        car.reset_position()
        car.get_odometry(True)
        car.get_distance(True)

    @staticmethod
    def _default_before_any(car, task_name):
        """任务触发后、run(car) 执行前的通用提示: 蜂鸣 + 打日志."""
        logger.info(f"触发任务点: {task_name}")
        try:
            car.beep()
        except Exception:
            pass

    # ------------------------------------------------------------------

    def __init__(self, car):
        """
        初始化编排器。

        参数:
            car: MyCar 实例，用于读取按键（car.key）
        """
        self.car = car
        self.done = set()  # 本次运行中已确认完成的任务名集合
        self.running = True  # 当前运行流程是否还在进行
        self._key = car.key  # Key4Btn 实例（内部可能换实现，直接使用接口）
        self._skip_listener = None  # 任务运行期间的跳过监听线程
        self._skipped = False  # 当前任务是否被跳过
        self._emergency = False  # 是否发生急停

        # 按键事件使用队列：由独立按键线程写入，主流程消费
        self._key_queue = []
        self._key_lock = threading.Lock()
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        # 运行时的触发条件覆盖表: key=task_name, value=合并后的 config
        # 调用 override_trigger(task_name, **kwargs) 写入
        self._trigger_overrides: Dict[str, Dict] = {}

        # 运行时的 before/after 钩子注册表
        self._before_hooks: Dict[str, Callable] = {
            # 默认: 所有任务触发后先 beep + 打日志
            "__all__": self._default_before_any,
        }
        self._after_hooks: Dict[str, Callable] = {
            "sorting": self._default_after_sorting,
        }

        # 上一次 cruise_to_trigger 的结果（调试/排错用）
        self.last_cruise_result: Dict = {}

    # ------------------------------------------------------------------
    # 按键采集线程
    # ------------------------------------------------------------------
    def _key_loop(self):
        """独立线程轮询按键事件，写入内部队列。"""
        while self.running:
            try:
                # get_btn() 返回 0 表示无事件，1~12 表示事件码（短按 1~4、长按 5~8、连按 9~12）
                event = self._key.get_btn()
                if event and event <= 4:  # 只处理短按，避免长按/连按误触发
                    with self._key_lock:
                        self._key_queue.append(event)
            except Exception:
                # 按键读取异常时静默跳过，避免编排线程崩溃
                pass
            time.sleep(0.02)

    def _pop_key(self):
        """取出一条按键事件；无事件返回 0。"""
        with self._key_lock:
            if self._key_queue:
                return self._key_queue.pop(0)
        return 0

    def _flush_keys(self):
        """清空当前按键队列（用于进入任务前丢弃残留按键）。"""
        with self._key_lock:
            self._key_queue = []

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------
    def wait_start(self):
        """阻塞等待一键启动（按键 4）。返回时表示本轮运行开始。"""
        print("等待一键启动...（按 4 开始，按 3 急停退出）")
        self._flush_keys()
        while self.running:
            key = self._pop_key()
            if key == self.KEY_START:
                print("一键启动!")
                return
            if key == self.KEY_EMERGENCY:
                print("启动前急停，退出流程")
                self.car._stop_flag = True
                self.car.stop()
                raise SystemExit("启动前急停")
            time.sleep(0.05)

    def schedule(self):
        """
        按顺序产出本轮需要执行的任务名。

        重来时跳过本次运行中已完成的"继续"；跳过/未完成的都会产出。
        若所有任务都已完成，则产出空列表（编排结束）。
        """
        return [t for t in self.TASK_ORDER if t not in self.done]

    def _listen_skip(self):
        """
        任务运行期间的跳过/急停监听线程。

        因为任务函数内部没有编排器相关的打断点，这里通过停止车辆（car.stop()）
        让任务函数从运动原语中尽快返回，并记录跳过的任务名供主流程判断。
        急停（按键 3）则额外置位 MyCar 的 _stop_flag，运动原语会立即中断。
        """
        while self._skip_listener and not self._skip_listener.is_set():
            key = self._pop_key()
            if key == self.KEY_SKIP:
                print("=== 跳过当前任务 ===")
                self._skipped = True
                self.car.stop()
                return
            if key == self.KEY_EMERGENCY:
                print("=== 急停 ===")
                self._emergency = True
                # 复用 MyCar 的急停标志：运动原语检测到 _stop_flag 后立即停下
                self.car._stop_flag = True
                self.car.stop()
                return
            time.sleep(0.02)

    def start_skip_listener(self):
        """启动任务运行期间的跳过/急停监听线程。"""
        self._flush_keys()
        self._skipped = False
        self._emergency = False
        self._skip_listener = threading.Event()
        t = threading.Thread(target=self._listen_skip, daemon=True)
        t.start()

    def stop_skip_listener(self):
        """停止跳过/急停监听线程，返回 (是否跳过, 是否急停)。"""
        if self._skip_listener is not None:
            self._skip_listener.set()
            self._skip_listener = None
        return self._skipped, self._emergency

    def mark_done(self, task_name):
        """任务执行结束后标记为已完成（本次运行内生效）。"""
        self.done.add(task_name)
        print(f"任务完成: {task_name}  （已完成: {sorted(self.done)}）")

    def abort(self):
        """急停/终止编排：结束按键线程。"""
        self.running = False
        try:
            self._key_thread.join(timeout=1.0)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # 触发配置 & 钩子管理
    # ------------------------------------------------------------------
    def override_trigger(self, task_name: str, **kwargs):
        """运行时覆盖某个任务的触发参数（不修改类默认值）。

        示例:
            orch.override_trigger("seeding", distance=1.1)
            orch.override_trigger("target_detection", min_score=0.55, confirm=5)
        """
        if task_name not in self.TASK_TRIGGER:
            raise ValueError(
                f"未知任务名 {task_name}; 可选: {sorted(self.TASK_TRIGGER.keys())}"
            )
        merged = dict(self.TASK_TRIGGER[task_name])
        merged.update(kwargs)
        self._trigger_overrides[task_name] = merged

    def set_before_hook(self, task_name: str, hook: Optional[Callable]):
        """设置任务触发后、执行 run(car) 前的钩子.

        参数:
            task_name: 任务名, 或 "__all__" 表示所有任务都会调用的通配钩子
                       (通配钩子会先于任务专属钩子执行)
            hook:      callable(car, task_name) -> None; 传 None 删除
        """
        if hook is None:
            self._before_hooks.pop(task_name, None)
        else:
            self._before_hooks[task_name] = hook

    def set_after_hook(self, task_name: str, hook: Optional[Callable]):
        """设置任务 run(car) 执行完毕后的钩子 (sorting 已有默认清零钩子)."""
        if hook is None:
            self._after_hooks.pop(task_name, None)
        else:
            self._after_hooks[task_name] = hook

    def clear_hooks(self):
        """清空所有 before/after 钩子 (默认钩子也会被清掉, 慎用)."""
        self._before_hooks.clear()
        self._after_hooks.clear()

    def _resolve_trigger_config(self, task_name: str) -> Dict:
        """返回最终生效的触发配置 (默认值 + 运行时覆盖合并)."""
        cfg = dict(self.TASK_TRIGGER[task_name])
        if task_name in self._trigger_overrides:
            cfg.update(self._trigger_overrides[task_name])
        cfg.setdefault("use_stop", True)
        cfg.setdefault("start_dist", 0.0)
        cfg.setdefault("max_run", 0.0)
        cfg.setdefault("time_out", 0.0)
        cfg.setdefault("speed", 0.3)
        if cfg["type"] == "vision":
            cfg.setdefault("min_score", 0.5)
            cfg.setdefault("confirm", 3)
            cfg.setdefault("fresh", False)
            cfg.setdefault("max_age", 0.3)
        return cfg

    def _call_before_hooks(self, task_name: str):
        """按顺序调用 "__all__" 通配 + 任务专属 before 钩子."""
        funcs: List[Callable] = []
        if "__all__" in self._before_hooks:
            funcs.append(self._before_hooks["__all__"])
        if task_name in self._before_hooks and task_name != "__all__":
            funcs.append(self._before_hooks[task_name])
        for fn in funcs:
            try:
                fn(self.car, task_name)
            except Exception as e:
                logger.warning(f"before_hook[{task_name}] 异常: {e}")

    def _call_after_hooks(self, task_name: str):
        """调用任务专属 after 钩子 (sorting 默认清零里程计)."""
        if task_name in self._after_hooks:
            try:
                self._after_hooks[task_name](self.car, task_name)
            except Exception as e:
                logger.warning(f"after_hook[{task_name}] 异常: {e}")

    # ------------------------------------------------------------------
    # 触发判定原语
    # ------------------------------------------------------------------
    def _check_vision_trigger(
        self, cfg: Dict, confirm_buf: List[str]
    ) -> Tuple[bool, Optional[str], List]:
        """侧视实时检测视觉触发.

        参数:
            cfg:          解析后的 TASK_TRIGGER 子配置 (vision 类型)
            confirm_buf:  引用外部的列表, 记录连续命中帧的 label;
                          本函数内部 append/pop，保持 0..confirm 长度滑动

        返回:
            (confirmed, matched_label, dets_matched)
                confirmed=True 表示已达到连续 confirm 帧命中
        """
        labels = set(cfg["labels"])
        min_score = float(cfg["min_score"])
        confirm = int(cfg["confirm"])

        # 取侧视检测结果 (fresh / cached)
        try:
            if cfg["fresh"]:
                dets = self.car.get_realtime_detections(fresh=True)
            else:
                dets = self.car.get_realtime_detections(
                    fresh=False, max_age=cfg["max_age"]
                )
        except Exception as e:
            logger.warning(f"视觉触发取检测结果异常: {e}")
            dets = []

        matched_label: Optional[str] = None
        matched_dets = []
        for det in dets:
            # 格式: [cls_id, obj_id, label, score, x_c, y_c, w, h]
            try:
                det_label, det_score = str(det[2]), float(det[3])
            except Exception:
                continue
            if det_label in labels and det_score >= min_score:
                matched_label = det_label
                matched_dets.append(det)
                break  # 只要有一个命中即可确认本帧

        # 维护"连续命中帧"缓冲
        if matched_label is not None:
            confirm_buf.append(matched_label)
            if len(confirm_buf) > confirm:
                confirm_buf.pop(0)
        else:
            confirm_buf.clear()

        confirmed = len(confirm_buf) >= confirm and len(confirm_buf) > 0
        if confirmed:
            matched_label = confirm_buf[-1]
        return confirmed, matched_label, matched_dets

    @staticmethod
    def _check_odometer_trigger(
        cfg: Dict, start_distance: float, current_distance: float
    ) -> bool:
        """里程计触发判定: 当前累计距离 - 起点 >= cfg["distance"] 即 True.

        注意: 这里用的是 get_distance() (累计路程, 非直线位移),
        与 lane_dis_offset 的停止判定一致, 避免因横移/转弯导致距离偏差.
        """
        rel = current_distance - start_distance
        return rel >= float(cfg["distance"])

    # ------------------------------------------------------------------
    # 巡线 → 触发点 主方法
    # ------------------------------------------------------------------
    def cruise_to_trigger(self, task_name: str, **cfg_override) -> Dict:
        """从当前位置起, 用前置摄像头巡线 (lane_base) 前进, 持续检测该任务的
        触发条件 (视觉/里程计)，命中即停车.

        公共兜底:
            - 行驶距离超过 cfg["max_run"] (若 >0) 强制停, 防止过站
            - 用时超过 cfg["time_out"] (若 >0) 强制停
            - 按键 3 急停: 复用 MyCar._stop_flag, 检测到后立即返回
            - 按键 1 跳过: 设置 self._skipped=True 后返回

        参数:
            task_name:    任务名, 必须在 TASK_TRIGGER 中
            cfg_override: 临时覆盖本次巡航的任何触发参数 (一次性, 不改注册表)

        返回:
            dict 含字段:
                ok:           bool, 是否按触发条件正常命中 (否则是兜底/跳过/急停)
                reason:       str,  "vision" / "odometer" / "max_run" / "time_out"
                                    / "skip" / "emergency" / "error"
                matched_label:str,  vision 命中时的 label, 否则 None
                traveled:     float,本次巡航累计路程 (米, 相对起点)
                duration:     float,本次巡航用时 (秒)
                start_dist:   float,巡航起点累计路程
                start_odom:   list, 巡航起点位姿 [x,y,theta]
        """
        if task_name not in self.TASK_TRIGGER:
            raise ValueError(
                f"未知任务名 {task_name}; 可选: {sorted(self.TASK_TRIGGER.keys())}"
            )
        cfg = self._resolve_trigger_config(task_name)
        cfg.update(cfg_override)

        car = self.car
        start_distance = float(car.get_distance())
        start_odom = list(car.get_odometry())
        start_time = time.time()

        confirm_buf: List[str] = []

        # 返回结果占位
        result: Dict = {
            "ok": False,
            "reason": "error",
            "matched_label": None,
            "traveled": 0.0,
            "duration": 0.0,
            "start_dist": start_distance,
            "start_odom": start_odom,
        }

        # 清空按键队列, 准备在巡航期间监听 skip/emergency
        self._flush_keys()
        self._skipped = False
        self._emergency = False

        # 停止判定 (每 tick 都调用)
        def _should_stop() -> Tuple[bool, str, Optional[str]]:
            # 1. 急停 (用户直接按 3 或 任务级监听都可置位)
            if getattr(car, "_stop_flag", False) or self._emergency:
                return True, "emergency", None
            # 2. 跳过
            if self._skipped:
                return True, "skip", None
            # 3. 先开 start_dist 窗口: 没走到 start_dist 就不检查触发
            cur_dist = float(car.get_distance())
            rel = cur_dist - start_distance
            if rel < float(cfg["start_dist"]):
                pass
            else:
                # 4. 触发判定
                if cfg["type"] == "vision":
                    confirmed, label, _dets = self._check_vision_trigger(
                        cfg, confirm_buf
                    )
                    if confirmed:
                        return True, "vision", label
                else:  # odometer
                    if self._check_odometer_trigger(cfg, start_distance, cur_dist):
                        return True, "odometer", None
            # 5. 兜底: max_run
            if float(cfg["max_run"]) > 0 and rel >= float(cfg["max_run"]):
                logger.warning(f"[{task_name}] 触发兜底: 已行驶 {rel:.3f}m >= max_run")
                return True, "max_run", None
            # 6. 兜底: time_out
            if float(cfg["time_out"]) > 0:
                if time.time() - start_time > float(cfg["time_out"]):
                    logger.warning(
                        f"[{task_name}] 触发兜底: 用时 {time.time()-start_time:.1f}s 超时"
                    )
                    return True, "time_out", None
            return False, "", None

        # 7. 在巡航期间用轻量轮询消费按键队列 (跳过/急停, 不占串口)
        cruise_stop = threading.Event()

        def _key_watcher():
            while not cruise_stop.is_set():
                key = self._pop_key()
                if key == self.KEY_SKIP:
                    print(f"=== 巡航跳过: {task_name} ===")
                    self._skipped = True
                    car.stop()
                    return
                if key == self.KEY_EMERGENCY:
                    print(f"=== 巡航急停: {task_name} ===")
                    self._emergency = True
                    car._stop_flag = True
                    car.stop()
                    return
                time.sleep(0.02)

        key_w = threading.Thread(target=_key_watcher, daemon=True)
        key_w.start()

        try:
            speed = float(cfg["speed"])
            # 用 lane_base 闭环巡线前进; 若 lane_base 异常, 退化为开环 move_base
            try:
                car.lane_base(
                    speed,
                    end_fuction=lambda: _should_stop()[0],
                    stop=cfg["use_stop"],
                )
            except Exception as e:
                logger.warning(f"lane_base 异常, 退化为开环前进: {e}")
                car.move_base(
                    [speed, 0.0, 0.0],
                    end_fuction=lambda: _should_stop()[0],
                    stop=cfg["use_stop"],
                )
        finally:
            cruise_stop.set()
            try:
                key_w.join(timeout=0.5)
            except Exception:
                pass

        # 最终再收一次结果 (只要 reason / label, ok 没单独用, 直接 reason 判断)
        _ok, reason, label = _should_stop()
        # 如果 lane_base 正常结束但没有命中任何 reason (几乎不会发生), 记为 max_run
        if not reason:
            reason = "max_run"
        if cfg.get("use_stop", True):
            car.stop()

        cur_dist = float(car.get_distance())
        result["ok"] = reason in {"vision", "odometer"}
        result["reason"] = reason
        result["matched_label"] = label
        result["traveled"] = cur_dist - start_distance
        result["duration"] = time.time() - start_time
        self.last_cruise_result = result
        logger.info(
            f"cruise_to_trigger[{task_name}] ok={result['ok']} reason={reason} "
            f"label={label} traveled={result['traveled']:.3f}m dur={result['duration']:.1f}s"
        )
        return result

    # ------------------------------------------------------------------
    # 任务模块调用
    # ------------------------------------------------------------------
    @staticmethod
    def _import_task_module(task_name: str):
        """延迟加载 tasks.<task_name> 模块 (避免循环 import)."""
        return importlib.import_module(f"tasks.{task_name}")

    def run_task_module(self, task_name: str, *extra_args, **extra_kwargs):
        """调用 tasks.<task_name>.run(car, *extra_args, **extra_kwargs).

        对 shooting/delivery 这类接受额外参数的任务, 可在调用时传入:
            orch.run_task_module("shooting", [0,0,1,0])
        返回值就是任务模块 run() 的返回.
        """
        mod = self._import_task_module(task_name)
        # 清空任务级别的急停标志, 让任务可以正常跑
        self.car._stop_flag = False
        return mod.run(self.car, *extra_args, **extra_kwargs)

    # ------------------------------------------------------------------
    # 一键编排: 启动→逐个任务巡线触发→(可选)执行→钩子→标记
    # ------------------------------------------------------------------
    def run_next_task(
        self,
        auto_run: bool = True,
        task_args: Optional[Dict[str, Tuple]] = None,
        task_kwargs: Optional[Dict[str, Dict]] = None,
    ) -> Tuple[bool, str, Dict, object]:
        """调度下一个任务（按 schedule() 顺序）: 巡线触发 → before → run → after → mark_done.

        参数:
            auto_run:   True 会自动调任务的 run(car); False 只做巡线触发+钩子,
                        由外部决定何时 run (配合回调钩子也可以在钩子里跑)
            task_args:  可选 {task_name: (arg1,...)} 传给 run() 的位置参数
            task_kwargs:可选 {task_name: {kw:val}} 传给 run() 的关键字参数

        返回:
            (has_more, task_name, cruise_result, task_return)
                has_more=False 表示没有剩余任务
        """
        pending = self.schedule()
        if not pending:
            return False, "", {}, None
        task_name = pending[0]
        task_args = task_args or {}
        task_kwargs = task_kwargs or {}

        # 1) 巡线到触发点
        cruise_res = self.cruise_to_trigger(task_name)
        # 急停/跳过 → 不标记完成, 不执行
        if cruise_res["reason"] == "emergency":
            print(f"[run_all] {task_name} 巡航急停, 终止编排")
            self.running = False
            return False, task_name, cruise_res, None
        if cruise_res["reason"] == "skip":
            print(f"[run_all] {task_name} 被跳过 (本次不标记, 下次重来可补做)")
            # 注意: schedule() 下次重来还会再产出, 与按键语义一致
            return True, task_name, cruise_res, None

        # 2) before 钩子 (触发后, 执行 run() 前)
        self._call_before_hooks(task_name)

        # 3) 可选: 调用任务 run(car)
        task_return = None
        if auto_run:
            args = task_args.get(task_name, ())
            kwargs = task_kwargs.get(task_name, {})
            # 任务函数执行期间再打开 skip/emergency 监听 (复用原有的 start_skip_listener)
            self.start_skip_listener()
            try:
                task_return = self.run_task_module(task_name, *args, **kwargs)
            finally:
                skipped, emergency = self.stop_skip_listener()
                if emergency:
                    print(f"[run_all] {task_name} 执行期间急停, 终止编排")
                    self.running = False
                    return False, task_name, cruise_res, task_return
                if skipped:
                    print(f"[run_all] {task_name} 执行期间被跳过 (不标记完成)")
                    return True, task_name, cruise_res, task_return

        # 4) after 钩子 (sorting 默认清零里程计, 让 ordering 的触发基点正确)
        self._call_after_hooks(task_name)

        # 5) 标记完成 → schedule() 下一轮重来会跳过
        self.mark_done(task_name)
        return True, task_name, cruise_res, task_return

    def run_all(
        self,
        auto_run_task: bool = True,
        wait_start: bool = True,
        allow_restart: bool = True,
        task_args: Optional[Dict[str, Tuple]] = None,
        task_kwargs: Optional[Dict[str, Dict]] = None,
    ) -> List[Tuple[str, Dict, object]]:
        """完整比赛流程: 按键 4 启动 → 按顺序巡线触发 8 个任务 → 标记 → 结束.

        参数:
            auto_run_task:  True (默认) = 触发后自动调用各任务 run(car)
                            False = 只做"巡线触发 + 钩子 + 标记", 不执行任务本体
            wait_start:     True (默认) = 进入前阻塞等待按键 4 启动
            allow_restart:  True (默认) = 所有任务结束或急停后, 回到 wait_start
                            等按键 4 重来 (重来时跳过已 mark_done 的任务);
                            False = 跑完一轮就 return
            task_args / task_kwargs: 透传给 run_next_task, 给特定任务喂额外参数
        返回:
            list of (task_name, cruise_result, task_return)
        """
        run_log: List[Tuple[str, Dict, object]] = []
        while True:
            if wait_start:
                try:
                    self.wait_start()
                except SystemExit:
                    return run_log
                if not self.running:
                    return run_log

            while self.running:
                has_more, name, cru_res, t_ret = self.run_next_task(
                    auto_run=auto_run_task,
                    task_args=task_args,
                    task_kwargs=task_kwargs,
                )
                if name:
                    run_log.append((name, cru_res, t_ret))
                if not has_more:
                    # 没有剩余任务 or 急停终止
                    break

            if not allow_restart:
                return run_log
            if not self.running:
                # 急停: 让用户按 4 之后能重来 → 恢复 running, 清掉急停标志
                self.running = True
                try:
                    self.car._stop_flag = False
                except Exception:
                    pass
            # 否则 (所有任务已完成 / 被跳过): 给用户一个可视反馈, 然后回等按键 4
            if not self.schedule():
                print("===== 本轮全部任务已完成 =====")
                try:
                    self.car.beep()
                    self.car.beep()
                except Exception:
                    pass
            # 下一轮重新 wait_start (按键 4 = 重来, 已 done 的按语义会跳过)
            wait_start = True
