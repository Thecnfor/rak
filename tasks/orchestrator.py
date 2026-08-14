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
"""
import threading
import time


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
