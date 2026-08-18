# -*- coding: utf-8 -*-
"""一键编排: run_next_task / run_all / run_task_module.

和 Orchestrator 的关系:
    Orchestrator 继承实现了 RunnerHost (下面定义的最小接口),
    TaskRunner 把所有"按顺序走 8 个任务"的业务逻辑包起来.
    用户仍然只和 Orchestrator 门面交互, 不需关心这里面有几层.
"""
import importlib
from typing import Dict, List, Optional, Tuple

from .trigger_configs import TASK_ORDER, TriggerConfigManager
from .cruiser import Cruiser
from .hooks import HookManager


# =========================================================================
# TaskRunner 需要 Orchestrator 提供的最小能力集合
# =========================================================================
class RunnerHost:
    car: object
    running: bool
    KEY_SKIP: int = 1
    KEY_EMERGENCY: int = 3

    # 按键
    def _pop_key(self) -> int: ...
    def _flush_keys(self) -> None: ...

    # 跳过/急停
    _skipped: bool
    _emergency: bool

    def start_skip_listener(self) -> None: ...
    def stop_skip_listener(self) -> Tuple[bool, bool]: ...

    # 编排
    def schedule(self) -> List[str]: ...
    def mark_done(self, _task_name: str) -> None: ...
    def wait_start(self) -> None: ...
    def abort(self) -> None: ...


# =========================================================================
# 任务模块调用
# =========================================================================
def import_task_module(task_name: str):
    """延迟加载 tasks.<task_name> 模块 (避免循环 import)."""
    assert isinstance(task_name, str) and len(task_name) > 0
    if task_name not in TASK_ORDER:
        raise ValueError(f"未知任务名 {task_name}; 可选: {sorted(TASK_ORDER)}")
    return importlib.import_module("tasks." + task_name)


def run_task_module(host: RunnerHost, _task_name: str, *args, **kwargs):
    """调用 tasks.<_task_name>.run(car, *args, **kwargs), 返回它的返回值."""
    mod = import_task_module(_task_name)
    # 清急停标志, 让任务能正常运动 (任务结束/被打断再重新置位都 OK)
    try:
        host.car._stop_flag = False
    except Exception:
        pass
    return mod.run(host.car, *args, **kwargs)


# =========================================================================
# 一键编排器
# =========================================================================
class TaskRunner:
    """把"等按键 → 逐个任务巡航触发 → (可选)执行 → 钩子 → 标记" 包起来."""

    def __init__(
        self,
        host: RunnerHost,
        cfg_manager: TriggerConfigManager,
        cruiser: Cruiser,
        hooks: HookManager,
    ) -> None:
        self.host = host
        self.cfg_manager = cfg_manager
        self.cruiser = cruiser
        self.hooks = hooks
        # 已执行任务的 run() 返回值 {task_name: return}, 供下游任务结果链接力取用
        self.results: Dict[str, object] = {}

    # ---------- 任务结束钉姿势 ----------
    def _pin_end_pose(self, task_name: str) -> None:
        """任务 run() 结束后, 若配置了 end_pose 则 go_to_pose 到该绝对位姿.

        放在 after 钩子之前调用 (sorting 的清里程计钩子发生在钉姿势之后)。
        end_pose 语义: 当前里程计坐标系下的 [x, y, theta] 弧度。
        """
        cfg = self.cfg_manager.resolve(task_name)
        end_pose = cfg.get("end_pose")
        if not end_pose:
            return
        try:
            target = [float(end_pose[0]), float(end_pose[1]), float(end_pose[2])]
            ok = self.host.car.go_to_pose(target)
            print(
                f"[run_all] {task_name} 钉姿势 -> {[f'{v:.3f}' for v in target]} "
                f"{'OK' if ok else '超时/失败'}"
            )
        except Exception as e:
            print(f"[run_all] {task_name} 钉姿势异常: {e}")

    # ---------- 单步 ----------
    def run_next_task(
        self,
        auto_run: bool = True,
        task_args: Optional[Dict[str, Tuple]] = None,
        task_kwargs: Optional[Dict[str, Dict]] = None,
    ) -> Tuple[bool, str, Dict, object]:
        """调度并执行下一个任务 (按 schedule() 顺序).

        返回:
            (has_more, task_name, cruise_result, task_return)
        """
        pending = self.host.schedule()
        if not pending:
            return False, "", {}, None
        task_name = pending[0]
        task_args = task_args or {}
        task_kwargs = task_kwargs or {}

        # 1) 巡线到触发点
        cruise_res = self.cruiser.cruise_to_trigger(task_name)
        if cruise_res["reason"] == "emergency":
            print(f"[run_all] {task_name} 巡航急停, 终止编排")
            self.host.running = False
            return False, task_name, cruise_res, None
        if cruise_res["reason"] == "skip":
            print(f"[run_all] {task_name} 被跳过 (不标记完成, 下次重来仍补做)")
            return True, task_name, cruise_res, None

        # 2) before 钩子
        self.hooks.call_before(self.host.car, task_name)

        # 3) 可选: 调用任务 run(car)
        task_return = None
        if auto_run:
            args = task_args.get(task_name, ())
            kwargs = dict(task_kwargs.get(task_name, {}))  # 拷贝, 避免污染调用方配置
            # 结果链接力: kwargs 值为 callable 时, 用已完成任务的返回值表实时求值
            # (例: {"animal_list": lambda r: r["target_detection"]} 把上一个任务
            #  的返回值作为本任务入参, 任务未跑/被跳过时回落默认 None)
            for k, v in list(kwargs.items()):
                if callable(v):
                    kwargs[k] = v(self.results)
            self.host.start_skip_listener()
            try:
                task_return = run_task_module(self.host, task_name, *args, **kwargs)
                self.results[task_name] = task_return
            finally:
                skipped, emergency = self.host.stop_skip_listener()
                if emergency:
                    print(f"[run_all] {task_name} 执行期间急停, 终止编排")
                    self.host.running = False
                    return False, task_name, cruise_res, task_return
                if skipped:
                    print(f"[run_all] {task_name} 执行期间被跳过 (不标记完成)")
                    return True, task_name, cruise_res, task_return

        # 4) 任务结束钉姿势: 无论车停在哪个姿势, 都 go_to_pose 到配置的
        #    绝对 end_pose, 让下一个任务从已知姿势开始 (放在 after 钩子前,
        #    确保 sorting 的清里程计钩子发生在钉姿势之后)
        self._pin_end_pose(task_name)

        # 5) after 钩子 (sorting 默认清零里程计, 为 ordering 做起点)
        self.hooks.call_after(self.host.car, task_name)

        # 6) 标记完成 (schedule() 下次就跳过它)
        self.host.mark_done(task_name)
        return True, task_name, cruise_res, task_return

    # ---------- 一键串起 ----------
    def run_all(
        self,
        auto_run_task: bool = True,
        wait_start: bool = True,
        allow_restart: bool = True,
        task_args: Optional[Dict[str, Tuple]] = None,
        task_kwargs: Optional[Dict[str, Dict]] = None,
    ) -> List[Tuple[str, Dict, object]]:
        """
        Args:
            auto_run_task:  True=触发后自动调 run(car); False=只做巡线+钩子+标记
            wait_start:     True=阻塞等按键 4 启动
            allow_restart:  True=一轮结束或急停后回到 wait_start 等重来
        Returns:
            [(task_name, cruise_result, task_return), ...]
        """
        run_log: List[Tuple[str, Dict, object]] = []

        while True:
            if wait_start:
                try:
                    self.host.wait_start()
                except SystemExit:
                    return run_log
                if not self.host.running:
                    return run_log

            while self.host.running:
                has_more, name, cru_res, t_ret = self.run_next_task(
                    auto_run=auto_run_task,
                    task_args=task_args,
                    task_kwargs=task_kwargs,
                )
                if name:
                    run_log.append((name, cru_res, t_ret))
                if not has_more:
                    break

            if not allow_restart:
                return run_log

            # allow_restart 分支
            if not self.host.running:
                # 急停终止: 清标志, 让用户按 4 之后能重来
                self.host.running = True
                try:
                    self.host.car._stop_flag = False
                except Exception:
                    pass

            # 整轮全部完成 / 全部跳过 → 蜂鸣提示
            if not self.host.schedule():
                print("===== 本轮全部任务已完成 =====")
                try:
                    self.host.car.beep()
                    self.host.car.beep()
                except Exception:
                    pass

            # 下一轮继续 wait_start (此时已 done 的会被 schedule() 跳过)
            wait_start = True
