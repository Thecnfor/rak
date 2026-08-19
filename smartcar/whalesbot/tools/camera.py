#!/usr/bin/python3
# -*- coding: utf-8 -*-

import threading
from multiprocessing import Process
import time

import cv2
import platform
import os, sys


from .log_wrap import logger


# ============================================================
# 默认重试/超时参数
# ------------------------------------------------------------
# 选择依据:
#   - V4L2 select() 默认 ~10s 超时, 3 次重试覆盖短暂 USB 抖动
#   - 总超时 30s 上界, 保证上层(create_car)在有限时间内拿到明确异常
#   - 重试间隔 1s 与原实现一致, 避免惊群
# 这些值通过构造函数可被覆盖, 老调用方零修改
# ============================================================
DEFAULT_INIT_MAX_RETRIES = 3      # 试读失败最大重试次数(不含首次)
DEFAULT_INIT_TOTAL_TIMEOUT = 30.0  # init() 总时间上界(秒)
DEFAULT_INIT_RETRY_INTERVAL = 1.0  # 重试间隔(秒)


class CameraInitError(RuntimeError):
    """摄像头初始化失败(打开/读首帧阶段, 在有界重试后仍失败时抛出)。

    属性:
        src:        设备路径(如 "/dev/cam3")或索引
        stage:      失败阶段 ("open" / "read_first_frame" / "device_missing" / "exception")
        attempts:   已尝试次数(从 1 开始)
        duration:   init() 累计耗时(秒)
        cause:      底层异常对象(可能为 None)
    """

    def __init__(self, src, stage, attempts, duration, cause=None):
        self.src = src
        self.stage = stage
        self.attempts = attempts
        self.duration = duration
        self.cause = cause
        msg = (
            "Camera init failed: src={src!r} stage={stage!r} "
            "attempts={attempts} duration={duration:.2f}s"
        ).format(src=src, stage=stage, attempts=attempts, duration=duration)
        if cause is not None:
            msg += " cause={cause!r}".format(cause=cause)
        super().__init__(msg)


class Camera:
    def __init__(self, index=1, width=640, height=480,
                 init_max_retries=None, init_total_timeout=None,
                 init_retry_interval=None):
        # if src ==0:
        #     self.src = "/dev/video0"
        # elif src == 1:
        #     self.src = "/dev/video1"

        self.width = width
        self.height = height
        self.index = index

        # 有界重试参数(老调用方不传时用模块级默认值)
        self._max_retries = (
            DEFAULT_INIT_MAX_RETRIES if init_max_retries is None
            else int(init_max_retries)
        )
        self._total_timeout = (
            DEFAULT_INIT_TOTAL_TIMEOUT if init_total_timeout is None
            else float(init_total_timeout)
        )
        self._retry_interval = (
            DEFAULT_INIT_RETRY_INTERVAL if init_retry_interval is None
            else float(init_retry_interval)
        )

        # self.src =src
        self.cap = None
        self.frame = None
        # 新的帧到达事件(事件驱动, 供实时推流轮询各帧使用)
        self.frame_ready = threading.Event()
        # 暂停标志
        self.pause_flag = False
        self.stop_flag = False
        # 运行时掉线标志: 初始化成功后, 后台线程 init() 失败时置 True,
        # 供调用方(realtime.py 已用 getattr 防 None)感知状态, 不伪装成 ready
        self.disconnected = False

        # init() 抛 CameraInitError 时, 不进入半初始化状态: cap 已被 release,
        # 后台线程不启动, 让上层决定是否中止
        self.init()
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # thread是否运行标志
        self.flag_thread = False
        self.start_back_thread()
        # self.start()

    def init(self):
        """打开摄像头并确认能读到首帧。

        有界重试: 在 _max_retries 次内成功则正常返回;
        超过 _max_retries 次或超过 _total_timeout 仍失败时抛 CameraInitError。
        每次失败都 release() 当前 VideoCapture, 下次循环重新创建。
        """
        start_time = time.time()
        attempt = 0

        while True:
            attempt += 1
            elapsed = time.time() - start_time

            # 总超时检查
            if elapsed >= self._total_timeout:
                raise CameraInitError(
                    src=getattr(self, "src", self.index),
                    stage="total_timeout",
                    attempts=attempt - 1,
                    duration=elapsed,
                )
            # 最大重试次数检查(已尝试 attempt-1 次仍来到这里, 不允许进入下一次循环)
            if attempt > self._max_retries + 1:
                raise CameraInitError(
                    src=getattr(self, "src", self.index),
                    stage=self._last_failure_stage or "unknown",
                    attempts=attempt - 1,
                    duration=elapsed,
                    cause=self._last_failure_cause,
                )

            self._last_failure_stage = None
            self._last_failure_cause = None

            try:
                if "Windows" in platform.platform():
                    self.src = self.index
                    self.cap = cv2.VideoCapture(self.src, cv2.CAP_DSHOW)
                else:
                    self.src = "/dev/cam" + str(self.index)
                    # 如果self.src不存在，则报错
                    if os.path.exists(self.src) == False:
                        logger.error("摄像头{}不存在".format(self.src))
                        self._last_failure_stage = "device_missing"
                        time.sleep(self._retry_interval)
                        continue
                    self.cap = cv2.VideoCapture(self.src)
                    # 校验: 部分 V4L2 设备 isOpened() 为真但随后 REQBUFS 失败
                    # (USB 掉线重枚举时常见), 需试读一帧确认真正可用
                    if self.cap is None or not self.cap.isOpened():
                        logger.error("摄像头{}打开失败, 重试中...".format(self.src))
                        self._last_failure_stage = "open"
                        if self.cap:
                            self.cap.release()
                            self.cap = None
                        time.sleep(self._retry_interval)
                        continue
                    try:
                        ok, _ = self.cap.read()
                    except Exception as e:
                        ok = False
                        self._last_failure_cause = e
                    if not ok:
                        logger.error(
                            "摄像头{}试读失败(设备可能掉线), 重试中...".format(self.src)
                        )
                        self._last_failure_stage = "read_first_frame"
                        self.cap.release()
                        self.cap = None
                        time.sleep(self._retry_interval)
                        continue
                break
            except CameraInitError:
                raise
            except Exception as e:
                logger.error("init:摄像头打开错误!")
                self._last_failure_stage = "exception"
                self._last_failure_cause = e
                try:
                    if self.cap is not None:
                        self.cap.release()
                        self.cap = None
                except Exception:
                    pass
                time.sleep(self._retry_interval)
                # 异常分支也受总超时约束(下一轮循环顶部检查)

    def start_back_thread(self):
        # 如果未开启线程，开启线程
        if not self.flag_thread:
            self.cap_thread = threading.Thread(target=self.update, args=())
            self.cap_thread.daemon = True
            self.cap_thread.start()
            self.flag_thread = True
        # 注: 不再 sleep 0.5 —— 读帧是后台异步线程, read() 自会等 self.frame 就绪,
        # 这里阻塞纯属拖慢 Car 初始化(前后两个摄像头各 0.5s)。

    def update(self):
        while True:
            if self.stop_flag:
                break
            if self.pause_flag:
                continue
            try:
                ret, frame = self.cap.read()
                if ret:
                    self.frame = frame
                    self.frame_ready.set()  # 唤醒等帧的推流线程
                else:
                    logger.error("read:读取图像错误!!!!")
                    self._reconnect()
            except Exception as e:
                # print(e)
                logger.error("exception:摄像头错误!!")
                self._reconnect()

    def _reconnect(self):
        """后台线程读帧失败后的恢复: 用与构造时相同的有界 init() 重试。

        init() 在重试上界内仍失败时抛 CameraInitError —— 后台线程捕获后
        进入 disconnected 状态(disconnected=True, frame 保持 None),
        不再无限重试也不伪装 ready。realtime.py 已用 getattr 防 None,
        上层感知到无新帧自行降级。
        """
        try:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
            self.init()
            self.set_size(self.width, self.height)
            self.disconnected = False
        except CameraInitError as e:
            logger.error("reconnect 失败, 进入 disconnected 状态: {}".format(e))
            self.disconnected = True
            # 退避, 避免后台线程在持续失败时高频占用 CPU
            # (不重启 init, 等待外部 close() 或重启进程)
            time.sleep(self._retry_interval)

    def set_size(self, width, height):
        self.width = width
        self.height = height
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def read(self):
        while self.frame is None:
            time.sleep(0.1)
        return self.frame

    def close(self):
        self.stop_flag = True
        # 等待进程结束
        self.cap_thread.join()
        logger.info("{} close".format(self.src))
        self.cap.release()


def main():
    camera = Camera(1, 640, 480)
    # logger.info("camera test")
    # start_time = time.time()
    while True:
        try:
            img = camera.read()
            # print(img.shape)
            cv2.imshow("img", img)
            key = cv2.waitKey(1)
            # cost_time = time.time() - start_time
            # start_time = time.time()
            # print("fps:", 1 / cost_time)
            if key == ord("q"):
                time.sleep(0.1)
                break
        except Exception as e:
            logger.error(e)
    camera.close()
    logger.info("over")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
