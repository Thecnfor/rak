# -*- coding: utf-8 -*-
"""
按键驱动的任务编排器（轻薄门面）。

说明（心智负担最小化）：
    这个文件只做 3 件事：
      1. 维护按键线程 + 按键队列（4=启动，1=跳过，3=急停）
      2. 维护"已完成任务集合 / running / skip_listener"等编排状态
      3. 把"巡线触发 / 配置覆盖 / 钩子 / 一键 run_all"等业务逻辑
         全部交给 tasks.start 子模块实现，这里只做薄封装。

    想改"哪个任务用什么触发 / 巡线怎么判断 / 钩子做什么 → 去 tasks/start/ 下对应文件；
    想改"按键采集 / 已完成集合 / 跳过监听线程" → 直接改本文件，不会碰业务代码。
"""
import threading
import time
from typing import Dict, List, Optional, Tuple

from tasks.start import (
    TASK_ORDER,
    TASK_TRIGGER,
    TriggerConfigManager,
    Cruiser,
    HookManager,
    TaskRunner,
)


class Orchestrator:
    """任务编排器：按键采集 + 编排状态 + 薄封装的 start 子模块。"""

    # ================ 静态配置（为了老代码的引用兼容，这里仍然作为类属性再暴露一次）============
    TASK_ORDER = TASK_ORDER
    TASK_TRIGGER = TASK_TRIGGER

    # 按键语义
    KEY_START = 4  # 一键启动 / 重来
    KEY_SKIP = 1  # 跳过当前任务（不标记完成，下次重来仍可补做）
    KEY_EMERGENCY = 3  # 急停

    # ====================================================================
    # 初始化
    # ====================================================================
    def __init__(self, car):
        self.car = car
        self.done: set = set()  # 本次运行内已完成的任务集合
        self.skip: set = set()  # 静态跳过的任务集合(整个流程不跑, 不算完成)
        self.running = True  # 当前编排流程是否还在进行
        self._key = car.key  # Key4Btn 实例

        # ----- 跳过/急停：任务运行期间的监听 -----
        self._skip_listener = None  # 任务运行期间跳过监听线程
        self._skipped = False  # 当前任务是否被跳过
        self._emergency = False  # 是否发生急停

        # ----- 按键事件队列：独立线程写，主流程消费 -----
        self._key_queue: List[int] = []
        self._key_lock = threading.Lock()
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        # ----- 子模块：配置覆盖 / 钩子 / 巡线 / 一键编排 -----
        self._cfg_manager = TriggerConfigManager()
        self._hooks = HookManager()
        self._cruiser = Cruiser(host=self, cfg_manager=self._cfg_manager)
        self._runner = TaskRunner(
            host=self,
            cfg_manager=self._cfg_manager,
            cruiser=self._cruiser,
            hooks=self._hooks,
        )

        # 兼容：保留原先字段，便于外部/历史代码直接读取上一次结果
        self.last_cruise_result: Dict = {}

    # ====================================================================
    # 按键采集线程（本文件保留的职责）
    # ====================================================================
    def _key_loop(self):
        """独立线程轮询按键，把短按事件写入内部队列（1~4）。"""
        while self.running:
            try:
                # read(): 0=无事件, 1~12=事件码(短按1~4/长按5~8/连按9~12);
                # Key4Btn 没有 get_btn, 只有 read() 才分发给后端 get_btn
                event = self._key.read()
                if event and event <= 4:
                    with self._key_lock:
                        self._key_queue.append(event)
            except Exception:
                pass
            time.sleep(0.02)

    def _pop_key(self) -> int:
        with self._key_lock:
            if self._key_queue:
                return self._key_queue.pop(0)
        return 0

    def _flush_keys(self) -> None:
        with self._key_lock:
            self._key_queue = []

    # ====================================================================
    # 对外基础接口（编排状态 / 调度 / 跳过监听）
    # ====================================================================
    def wait_start(self):
        """阻塞等待按键 4（启动/重来）。急停（按键 3）会 SystemExit 退出。"""
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

    def schedule(self) -> List[str]:
        """本轮需要执行的任务（顺序按 TASK_ORDER，跳过 self.skip 与 self.done。"""
        return [t for t in self.TASK_ORDER if t not in self.skip and t not in self.done]

    # --- 任务运行期间的跳过/急停监听：本文件保留 ---
    def _listen_skip(self):
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
                self.car._stop_flag = True
                self.car.stop()
                return
            time.sleep(0.02)

    def start_skip_listener(self):
        self._flush_keys()
        self._skipped = False
        self._emergency = False
        self._skip_listener = threading.Event()
        t = threading.Thread(target=self._listen_skip, daemon=True)
        t.start()

    def stop_skip_listener(self) -> Tuple[bool, bool]:
        if self._skip_listener is not None:
            self._skip_listener.set()
            self._skip_listener = None
        return self._skipped, self._emergency

    def mark_done(self, task_name: str) -> None:
        self.done.add(task_name)
        print(f"任务完成: {task_name}  （已完成: {sorted(self.done)}）")

    def abort(self) -> None:
        self.running = False
        try:
            self._key_thread.join(timeout=1.0)
        except Exception:
            pass

    # ====================================================================
    # 触发配置覆盖（转发给 TriggerConfigManager）
    # ====================================================================
    def override_trigger(self, task_name: str, **kwargs) -> None:
        """运行时覆盖任务触发参数（不会修改类默认值）。

        Example::

            orch.override_trigger("seeding", distance=1.1)
            orch.override_trigger("target_detection", min_score=0.55, confirm=5)
        """
        self._cfg_manager.override(task_name, **kwargs)

    # ====================================================================
    # 钩子（转发给 HookManager）
    # ====================================================================
    def set_before_hook(self, task_name: str, hook) -> None:
        """设置任务触发后、run(car) 前的钩子。
        task_name="__all__" 为通配，所有任务都会先执行通配再执行专属。
        hook=None 删除。
        """
        self._hooks.set_before(task_name, hook)

    def set_after_hook(self, task_name: str, hook) -> None:
        """任务 run(car) 之后的钩子。sorting 默认自带清零里程计钩子。"""
        self._hooks.set_after(task_name, hook)

    def clear_hooks(self) -> None:
        self._hooks.clear_all()

    # ====================================================================
    # 巡线 → 触发（转发给 Cruiser）
    # ====================================================================
    def cruise_to_trigger(self, task_name: str, **cfg_override) -> Dict:
        """从当前位置开始，lane_base 巡线前进，满足触发条件即停车。

        返回 dict:
            ok, reason(vision/odometer/max_run/time_out/skip/emergency/error),
            matched_label, traveled, duration, start_dist, start_odom
        """
        res = self._cruiser.cruise_to_trigger(task_name, **cfg_override)
        self.last_cruise_result = res
        return res

    # ====================================================================
    # 任务模块调用（转发给 TaskRunner / import_task_module）
    # ====================================================================
    @staticmethod
    def _import_task_module(task_name: str):
        from tasks.start.runner import import_task_module

        return import_task_module(task_name)

    def run_task_module(self, task_name: str, *extra_args, **extra_kwargs):
        """调用 tasks.<task_name>.run(car, *extra_args, **extra_kwargs)。"""
        from tasks.start.runner import run_task_module

        return run_task_module(self, task_name, *extra_args, **extra_kwargs)

    # ====================================================================
    # 一键串起来（转发给 TaskRunner）
    # ====================================================================
    def run_next_task(
        self,
        auto_run: bool = True,
        task_args: Optional[Dict[str, Tuple]] = None,
        task_kwargs: Optional[Dict[str, Dict]] = None,
    ) -> Tuple[bool, str, Dict, object]:
        """调度并执行下一个任务 (schedule()[0])。
        返回: (has_more, task_name, cruise_result, task_return)
        """
        return self._runner.run_next_task(
            auto_run=auto_run,
            task_args=task_args,
            task_kwargs=task_kwargs,
        )

    def run_all(
        self,
        auto_run_task: bool = True,
        wait_start: bool = True,
        allow_restart: bool = True,
        task_args: Optional[Dict[str, Tuple]] = None,
        task_kwargs: Optional[Dict[str, Dict]] = None,
    ) -> List[Tuple[str, Dict, object]]:
        """一键比赛流程。

        Args:
            auto_run_task:  True=触发后自动 run(car); False 只巡线+钩子+标记
            wait_start:    True=阻塞等按键 4
            allow_restart: True=一轮结束后回等按键 4 重来
        """
        return self._runner.run_all(
            auto_run_task=auto_run_task,
            wait_start=wait_start,
            allow_restart=allow_restart,
            task_args=task_args,
            task_kwargs=task_kwargs,
        )
