from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional

from .backend import CarBackend, EventType, TASK_ORDER


class RealBackend(CarBackend):
    """真后端：延迟初始化硬件，HRI 一键开始才占串口跑 run.py 标准流程。

    与 run.py 对齐：
      - 后端启动时**不** create_car（不占串口/摄像头/推理）
      - 用户点"开始"时才 _ensure_hardware() 初始化硬件 + Orchestrator
      - 一轮结束保持硬件占用等重来（run.py 的 allow_restart 语义）
      - 急停/退出时才释放硬件

    事件推送由本类在任务执行过程中手动 emit（task:started/done/skipped 等）。
    """

    def __init__(
        self,
        task_kwargs: Optional[Dict[str, Dict[str, Any]]] = None,
        stream: bool = True,
        static_skip: Optional[set] = None,
        after_hook=None,
    ) -> None:
        super().__init__()
        self._task_kwargs = task_kwargs or {}
        self._stream = stream
        # 静态跳过（run.py 的 SKIP_TASKS 兜底）
        self._static_skip: set = set(static_skip or set())
        # after 钩子（run.py 的 _pin_arm_and_reset）
        self._after_hook = after_hook

        # 延迟初始化的硬件/编排器
        self.orch = None
        self.car = None

        self._bg_thread: Optional[threading.Thread] = None
        self._odom_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._skip_flag = threading.Event()
        self._cmd_lock = threading.Lock()

        # 任务结果链：target_detection → shooting.animal_list / ordering → delivery.order_list
        self._task_results: Dict[str, Any] = {}

        # 每个任务的参数覆盖（lane PID / 触发参数），在 orch 初始化前保存，
        # start 时 _ensure_hardware() 应用到 orch.override_trigger
        self._task_config_overrides: Dict[str, Dict[str, Any]] = {}

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start_bg(self) -> None:
        if self._odom_thread and self._odom_thread.is_alive():
            return
        self._stop_flag.clear()
        self._odom_thread = threading.Thread(target=self._odom_loop, daemon=True)
        self._odom_thread.start()

    def close(self) -> None:
        self.stop()
        self._stop_flag.set()
        self._skip_flag.set()
        if self._bg_thread:
            self._bg_thread.join(timeout=5.0)
        if self._odom_thread:
            self._odom_thread.join(timeout=2.0)
        self._release_hardware()

    # ------------------------------------------------------------------
    # 硬件延迟初始化 / 释放
    # ------------------------------------------------------------------
    def _ensure_hardware(self) -> None:
        """点"开始"时才初始化硬件（create_car + Orchestrator + 钩子）。"""
        if self.car is not None and self.orch is not None:
            return
        from tasks.tools import create_car
        from tasks.orchestrator import Orchestrator

        car = create_car(reset=True, comp_mode=True, stream=self._stream)
        orch = Orchestrator(car)
        orch.skip = set(self._static_skip)
        if self._after_hook is not None:
            for task_name in TASK_ORDER:
                orch.set_after_hook(task_name, self._after_hook)
        # 应用之前保存的任务参数覆盖（lane PID / 触发参数）
        for task_name, cfg in self._task_config_overrides.items():
            try:
                orch.override_trigger(task_name, **cfg)
            except Exception as exc:
                print(f"[real_backend] 应用 {task_name} 参数覆盖失败: {exc}")
        self.car = car
        self.orch = orch
        print(f"[real_backend] 硬件已初始化 (create_car + Orchestrator, 任务数 {len(TASK_ORDER)})")

    def _release_hardware(self) -> None:
        """释放硬件（急停退出/服务关闭时）。"""
        car, self.car = self.car, None
        self.orch = None
        if car is not None:
            try:
                car.stop()
            except Exception:
                pass
            try:
                car.close()
            except Exception as exc:
                print(f"[real_backend] car.close 失败: {exc}")
            print("[real_backend] 硬件已释放")

    # ------------------------------------------------------------------
    # 命令
    # ------------------------------------------------------------------
    def start(self, from_index: int = -1) -> None:
        with self._cmd_lock:
            if self._active:
                raise RuntimeError("已有任务运行中")
            self._stop_flag.clear()
            self._skip_flag.clear()
            self._active = True

            # 点"开始"才初始化硬件（占串口）
            try:
                self._ensure_hardware()
            except Exception as exc:
                self._active = False
                self._emit({"type": EventType.ERROR, "message": f"硬件初始化失败: {exc}"})
                raise RuntimeError(f"硬件初始化失败: {exc}")

            # 应用触摸屏选中子集：未选中的任务加入 orch.skip（第一标准），
            # 与静态跳过（run.py SKIP_TASKS 兜底）取并集。
            self._apply_selection()

            start_idx = max(0, int(from_index)) if from_index >= 0 else 0
            self.orch.done.clear()
            # from_index 之前的任务先标记 done
            for i in range(start_idx):
                if i < len(TASK_ORDER):
                    self.orch.mark_done(TASK_ORDER[i])
            # 更新状态缓存
            for i, name in enumerate(TASK_ORDER):
                self._set_task_status(
                    name, "done" if i < start_idx else "pending"
                )

            self._emit({"type": EventType.RUN_STARTED})
            self._bg_thread = threading.Thread(
                target=self._run_all_worker, daemon=True
            )
            self._bg_thread.start()

    def _apply_selection(self) -> None:
        """把触摸屏选中子集应用到 orch.skip（未选中 = 跳过）。

        选中子集是"第一标准"；静态跳过（run.py SKIP_TASKS 兜底）始终保留。
        """
        skip = set(self._static_skip)
        if self._selected_tasks is not None:
            # 只跑选中的任务，未选中的跳过
            skip |= {t for t in TASK_ORDER if t not in self._selected_tasks}
        self.orch.skip = skip

    def run_task(self, name: str) -> None:
        if name not in TASK_ORDER:
            raise ValueError(f"未知任务: {name}")
        with self._cmd_lock:
            if self._active:
                raise RuntimeError("已有任务运行中")
            self._stop_flag.clear()
            self._skip_flag.clear()
            self._active = True
            try:
                self._ensure_hardware()
            except Exception as exc:
                self._active = False
                self._emit({"type": EventType.ERROR, "message": f"硬件初始化失败: {exc}"})
                raise RuntimeError(f"硬件初始化失败: {exc}")
            self._bg_thread = threading.Thread(
                target=self._run_single_worker, args=(name,), daemon=True
            )
            self._bg_thread.start()

    def stop(self) -> None:
        """急停：设置停止标志 + 注入按键 3 + 直接 car.stop。"""
        self._stop_flag.set()
        self._skip_flag.set()
        if self.orch is not None:
            try:
                with self.orch._key_lock:
                    self.orch._key_queue.append(self.orch.KEY_EMERGENCY)
            except Exception:
                pass
        if self.car is not None:
            try:
                self.car._stop_flag = True
                self.car.stop()
            except Exception:
                pass

    def skip(self) -> None:
        """跳过当前：设置跳过标志 + 注入按键 1。"""
        self._skip_flag.set()
        if self.orch is not None:
            try:
                with self.orch._key_lock:
                    self.orch._key_queue.append(self.orch.KEY_SKIP)
            except Exception:
                pass
        if self.car is not None:
            try:
                self.car.stop()
            except Exception:
                pass

    def reset(self) -> None:
        self.stop()
        time.sleep(0.2)
        with self._cmd_lock:
            if self.orch is not None:
                self.orch.done.clear()
            for name in TASK_ORDER:
                self._set_task_status(name, "pending")
            self._current_task = None
            self._active = False
            self._task_results.clear()
            self._stop_flag.clear()
            self._skip_flag.clear()
        self._emit({"type": EventType.RESET})

    def set_task_speed(self, name: str, speed: float) -> None:
        super().set_task_speed(name, speed)
        # 同步到编排器的触发覆盖（实际巡线速度由 lane.v_forward 决定，这里仅记元数据）
        clamped = max(0.05, min(2.0, float(speed)))
        if self.orch is not None:
            try:
                self.orch.override_trigger(name, speed=clamped)
            except Exception:
                pass

    def set_task_config(self, name: str, config: Dict[str, Any]) -> None:
        """设置某任务的参数覆盖（lane PID / 触发参数）。

        在 orch 初始化前调用会保存到 _task_config_overrides，start 时应用；
        orch 已初始化则立即 override_trigger。
        """
        if name not in TASK_ORDER:
            raise ValueError(f"未知任务: {name}")
        # 保存覆盖（供延迟初始化后应用）
        cur = self._task_config_overrides.setdefault(name, {})
        cur.update(config)
        # 若 orch 已初始化，立即应用
        if self.orch is not None:
            try:
                self.orch.override_trigger(name, **config)
            except Exception as exc:
                print(f"[real_backend] override_trigger {name} 失败: {exc}")

    def task_config_snapshot(self, name: str) -> Dict[str, Any]:
        """返回某任务的当前配置（默认 + 已保存的覆盖），供前端显示调节。"""
        base = super().task_config_snapshot(name)
        if name in self._task_config_overrides:
            base.update(self._task_config_overrides[name])
        return base

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------
    def current_odometry(self) -> Dict[str, float]:
        if self.car is None:
            return {"x": 0.0, "y": 0.0, "dist": 0.0}
        try:
            odom = self.car.get_odometry()
            dist = self.car.get_distance()
            x = float(getattr(odom, "x", 0.0) or 0.0)
            y = float(getattr(odom, "y", 0.0) or 0.0)
            return {
                "x": round(x, 3),
                "y": round(y, 3),
                "dist": round(float(dist or 0.0), 3),
            }
        except Exception:
            return {"x": 0.0, "y": 0.0, "dist": 0.0}

    # ------------------------------------------------------------------
    # Worker 线程
    # ------------------------------------------------------------------
    def _run_all_worker(self) -> None:
        try:
            while True:
                if self._stop_flag.is_set():
                    break
                schedule = self.orch.schedule()
                if not schedule:
                    break
                task_name = schedule[0]
                ret = self._execute_one(task_name)
                if self._stop_flag.is_set():
                    break
                if ret == "skipped" or ret == "emergency":
                    break
        finally:
            with self._cmd_lock:
                self._active = False
                self._current_task = None
            self._emit(
                {
                    "type": EventType.RUN_FINISHED,
                    "reason": "stopped" if self._stop_flag.is_set() else "completed",
                }
            )

    def _run_single_worker(self, name: str) -> None:
        try:
            self._execute_one(name)
        finally:
            with self._cmd_lock:
                self._active = False
                self._current_task = None
            self._emit({"type": EventType.RUN_FINISHED, "reason": "single_completed"})

    def _execute_one(self, task_name: str) -> str:
        """执行单个任务（巡线→before→run→after）。返回状态标记。"""
        if self._stop_flag.is_set():
            return "stopped"

        self._current_task = task_name
        self._set_task_status(task_name, "running")
        self._emit({"type": EventType.TASK_STARTED, "task": task_name})

        # 启动跳过/急停监听（复用 Orchestrator 的）
        self.orch.start_skip_listener()
        # 外部 skip 注入也要生效
        self._skip_flag.clear()

        # 注意：当后台监听命中 skip/emergency 时会写 orch._skipped / orch._emergency
        # 同时我们也监听 self._skip_flag / _stop_flag
        try:
            # --- 巡线到触发点 ---
            cruise_res = self.orch.cruise_to_trigger(task_name)
            if self._stop_flag.is_set() or self.orch._emergency:
                self._emit(
                    {
                        "type": EventType.TASK_SKIPPED,
                        "task": task_name,
                        "reason": "emergency",
                    }
                )
                self._set_task_status(task_name, "pending")
                return "emergency"
            if self._skip_flag.is_set() or self.orch._skipped:
                self._emit(
                    {
                        "type": EventType.TASK_SKIPPED,
                        "task": task_name,
                        "reason": "skip_requested",
                    }
                )
                self._set_task_status(task_name, "pending")
                return "skipped"
            cruise_ok = cruise_res.get("ok", False) if isinstance(cruise_res, dict) else False
            if not cruise_ok:
                reason = cruise_res.get("reason", "unknown") if isinstance(cruise_res, dict) else "cruise_failed"
                self._emit(
                    {
                        "type": EventType.TASK_ERROR,
                        "task": task_name,
                        "error": f"巡线触发失败: {reason}",
                    }
                )
                self._set_task_status(task_name, "failed")
                return "error"

            # --- before 钩子 ---
            try:
                self.orch._hooks.run_before(task_name, self.car)
            except Exception as exc:
                self._emit(
                    {
                        "type": EventType.TASK_ERROR,
                        "task": task_name,
                        "error": f"before钩子异常: {exc}",
                    }
                )
                self._set_task_status(task_name, "failed")
                return "error"

            # --- 运行任务模块 ---
            try:
                extra_kwargs = {}
                for k, v in (self._task_kwargs.get(task_name) or {}).items():
                    if callable(v):
                        try:
                            extra_kwargs[k] = v(self._task_results)
                        except Exception as exc2:
                            print(f"[real_backend] task_kwargs resolve {k} err: {exc2}")
                    else:
                        extra_kwargs[k] = v
                task_ret = self.orch.run_task_module(task_name, **extra_kwargs)
                self._task_results[task_name] = task_ret
            except Exception as exc:
                self._emit(
                    {
                        "type": EventType.TASK_ERROR,
                        "task": task_name,
                        "error": f"任务模块异常: {exc}",
                    }
                )
                self._set_task_status(task_name, "failed")
                return "error"

            if self._stop_flag.is_set() or self.orch._emergency:
                self._emit(
                    {
                        "type": EventType.TASK_SKIPPED,
                        "task": task_name,
                        "reason": "emergency",
                    }
                )
                self._set_task_status(task_name, "pending")
                return "emergency"
            if self._skip_flag.is_set() or self.orch._skipped:
                self._emit(
                    {
                        "type": EventType.TASK_SKIPPED,
                        "task": task_name,
                        "reason": "skip_requested",
                    }
                )
                self._set_task_status(task_name, "pending")
                return "skipped"

            # --- after 钩子 ---
            try:
                self.orch._hooks.run_after(task_name, self.car)
            except Exception as exc:
                # after 钩子异常不影响任务标记完成，但上报错误
                self._emit(
                    {
                        "type": EventType.ERROR,
                        "message": f"{task_name} after钩子异常: {exc}",
                    }
                )

            self.orch.mark_done(task_name)
            self._set_task_status(task_name, "done")
            self._emit({"type": EventType.TASK_DONE, "task": task_name})
            return "done"

        finally:
            _, _ = self.orch.stop_skip_listener()

    # ------------------------------------------------------------------
    # 里程计推送
    # ------------------------------------------------------------------
    def _odom_loop(self) -> None:
        while not self._stop_flag.is_set():
            try:
                odom = self.current_odometry()
                self._emit(
                    {
                        "type": EventType.ODOM,
                        "x": odom["x"],
                        "y": odom["y"],
                        "dist": odom["dist"],
                    }
                )
            except Exception:
                pass
            time.sleep(0.2)
