# -*- coding: utf-8 -*-
"""实时流与持续检测(RealtimeMixin): 来一帧推一帧, 背靠背推理(从 perception.py 拆分而来)。"""
import json
import threading
import time

import cv2
import zmq

from smartcar.whalesbot.tools import logger


class RealtimeMixin:

    # 侧视/前视 后台线程防重复启动标志
    _side_stream_flag = False
    _front_stream_flag = False

    # 巡线滤波系数(一阶低通 EMA, 0~1, 越大越跟随, 越小越平滑)
    _lane_ema = 0.35
    # 单次推理异常允许的最大时长(秒), 超时则按无误差直行处理
    _lane_timeout = 0.3

    def start_realtime_streams(self):
        """启动侧视(推理+推流)与前视(巡线推理+推流)共 4 个后台线程, 由 MyCar 初始化时调用。"""
        self._init_realtime_caches()
        self._start_side_streams()
        self._start_front_streams()

    def _start_side_streams(self):
        """启动侧视(cam2): 实时检测 + 实时推流两个线程。"""
        if RealtimeMixin._side_stream_flag:
            return
        RealtimeMixin._side_stream_flag = True
        threading.Thread(
            target=self._side_detect_loop, name="side_detect", daemon=True
        ).start()
        threading.Thread(
            target=self._side_stream_loop, name="side_stream", daemon=True
        ).start()

    def _start_front_streams(self):
        """启动前视(cam1): 实时巡线推理 + 实时推流两个线程。"""
        if RealtimeMixin._front_stream_flag:
            return
        RealtimeMixin._front_stream_flag = True
        threading.Thread(
            target=self._front_lane_loop, name="front_lane", daemon=True
        ).start()
        threading.Thread(
            target=self._front_stream_loop, name="front_stream", daemon=True
        ).start()

    def _init_realtime_caches(self):
        """初始化实时缓存(侧视检测 + 前视巡线)。"""
        self._det_lock = threading.Lock()
        self._det_cache = None  # (timestamp, dets)
        self._lane_lock = threading.Lock()
        self._lane_cache = None  # (timestamp, error, angle)
        self._lane_last = (0.0, 0.0)
        self._lane_last_ts = 0.0

    # ------------------------------------------------------------------
    # 侧视 task 检测接口
    # ------------------------------------------------------------------
    def get_realtime_detections(self, fresh=False, max_age=None):
        """实时获取侧视 task 检测结果。

        检测线程对最新帧背靠背推理并更新缓存; 本方法非阻塞返回最新结果。
        fresh=True 时立刻同步跑一次推理(独立连接, 不阻塞检测线程)。
        """
        if fresh:
            try:
                raw = self.cap_side.read()
            except Exception:
                return []
            if raw is None:
                return []
            sock = self._side_detect_client()
            try:
                dets = self._side_detect(sock, raw)
                with self._det_lock:
                    self._det_cache = (time.time(), dets)
            except Exception as e:
                logger.warning(f"实时检测(fresh)失败: {e}")
                return []
            finally:
                try:
                    sock.close(linger=0)
                except Exception:
                    pass
            return dets

        cache = self._get_det_cache()
        if cache is None:
            return []
        ts, dets = cache
        if max_age is not None and time.time() - ts > max_age:
            return []
        return dets

    def get_realtime_side_frame(self, with_overlay=True):
        """获取侧视最新画面; with_overlay 时在最新帧上实时叠加检测框。"""
        cache = self._get_det_cache()
        raw = getattr(self.cap_side, "frame", None)
        if not with_overlay or cache is None or raw is None or not cache[1]:
            return raw
        return self.draw_detection_results(raw, cache[1])

    def _get_det_cache(self):
        lock = getattr(self, "_det_lock", None)
        if lock is None:
            self._init_realtime_caches()
            lock = self._det_lock
        with lock:
            return self._det_cache

    def _side_detect_client(self):
        """创建独立于任务检测的 ZMQ 客户端(避免共享 socket 的线程竞争)。"""
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.RCVTIMEO = 5000
        sock.connect("tcp://127.0.0.1:5002")
        try:
            sock.send(b"ATATA")
            sock.recv()
        except Exception:
            pass
        return sock

    def _side_detect(self, sock, raw):
        """在侧视图上跑一次检测, 返回 [cls,obj,label,score, nx,ny,nw,nh] 列表。"""
        ok, buf = cv2.imencode(".jpg", raw)
        if not ok:
            return []
        sock.send(b"image" + buf.tobytes())
        res = json.loads(sock.recv())
        return res if isinstance(res, list) else []

    def _side_detect_loop(self):
        # 实时连续推理: 拿最新帧背靠背检测(节奏由推理速度决定, 无固定轮询)。
        # 结果写入 _det_cache, 供推流线程与 get_realtime_detections 使用。
        sock = None
        while not getattr(self, "_stop_flag", False):
            try:
                cap = self.cap_side
                raw = getattr(cap, "frame", None)
                if raw is None:
                    time.sleep(0.01)
                    continue
                if sock is None:
                    sock = self._side_detect_client()
                try:
                    dets = self._side_detect(sock, raw)
                    with self._det_lock:
                        self._det_cache = (time.time(), dets)
                except Exception as e:
                    logger.warning(f"侧视实时检测失败({e}), 重连中...")
                    try:
                        sock.close(linger=0)
                    except Exception:
                        pass
                    sock = None
            except Exception as e:
                logger.warning(f"侧视检测线程异常: {e}")

    def _side_stream_loop(self):
        # 事件驱动实时推流: 摄像头每抓一帧即被 frame_ready 唤醒并立即发布。
        # 若已有实时检测结果, 在最新帧上叠框后发布(满帧率出框); 无目标推原图。
        while not getattr(self, "_stop_flag", False):
            try:
                cap = self.cap_side
                ready = getattr(cap, "frame_ready", None)
                if ready is None:
                    time.sleep(1 / 60.0)
                    raw = cap.read()
                else:
                    ready.wait()
                    ready.clear()
                    raw = cap.frame
                if raw is None:
                    continue
                cache = self._get_det_cache()
                if cache is not None and cache[1]:
                    show = self.draw_detection_results(raw, cache[1])
                else:
                    show = raw
                self.streamer.update_frame(show, "cam2")
            except Exception as e:
                logger.warning(f"侧视流转发异常: {e}")
        RealtimeMixin._side_stream_flag = False

    # ------------------------------------------------------------------
    # 前视 lane 巡线接口
    # ------------------------------------------------------------------
    def _front_lane_loop(self):
        # 背靠背巡线推理: 拿最新前视帧跑 self.crusie(),
        # 结果做 EMA 平滑后写入 _lane_cache, 供推流线程绘制 + get_lane_results() 读取。
        while not getattr(self, "_stop_flag", False):
            try:
                cap = self.cap_front
                raw = getattr(cap, "frame", None)
                if raw is None:
                    time.sleep(0.01)
                    continue
                ts = time.time()
                try:
                    res = self.crusie(raw)
                    if not isinstance(res, (list, tuple)) or len(res) < 2:
                        raise ValueError(f"lane 推理结果异常: {res}")
                    error, angle = float(res[0]), float(res[1])
                except Exception as e:
                    logger.warning(f"前视巡线推理失败({e}), 保持上一帧")
                    last_ts = getattr(self, "_lane_last_ts", 0.0)
                    if time.time() - last_ts > self._lane_timeout:
                        error, angle = 0.0, 0.0
                    else:
                        error, angle = self._lane_last
                    # 异常帧只做缓存更新, 不做 EMA (否则异常叠加)
                    with self._lane_lock:
                        self._lane_cache = (ts, error, angle)
                    continue

                # 一阶低通滤波, 平滑单帧噪声
                l_e, l_a = self._lane_last
                error = l_e + self._lane_ema * (error - l_e)
                angle = l_a + self._lane_ema * (angle - l_a)
                self._lane_last = (error, angle)
                self._lane_last_ts = ts
                with self._lane_lock:
                    self._lane_cache = (ts, error, angle)
            except Exception as e:
                logger.warning(f"前视巡线线程异常: {e}")

    def _get_lane_cache(self):
        lock = getattr(self, "_lane_lock", None)
        if lock is None:
            self._init_realtime_caches()
            lock = self._lane_lock
        with lock:
            cache = self._lane_cache
        if cache is None:
            return 0.0, 0.0
        ts, error, angle = cache
        if time.time() - ts > self._lane_timeout:
            return 0.0, 0.0
        return error, angle

    @staticmethod
    def _draw_lane_overlay(image, error, angle):
        """在画面上绘制 d_e / d_a 描边文字(八邻域黑描边 + 中心绿字), 返回新图。"""
        label_text = f"d_e: {error:7.5f} d_a:{angle:7.5f}"
        # 用统一厚度偏移描边(黑边+绿字), 避免 cv2 5.x 下
        # 不同 thickness 渲染字形宽度不一致导致的白绿两层错位
        org = (20, 40)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                cv2.putText(
                    image,
                    label_text,
                    (org[0] + dx, org[1] + dy),
                    cv2.FONT_HERSHEY_TRIPLEX,
                    1.0,
                    (0, 0, 0),
                    1,
                    cv2.LINE_AA,
                )
        cv2.putText(
            image,
            label_text,
            org,
            cv2.FONT_HERSHEY_TRIPLEX,
            1.0,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    def _front_stream_loop(self):
        # 事件驱动前视推流: 摄像头每抓一帧即被 frame_ready 唤醒并立即发布到 cam1。
        # 每一帧都叠加 d_e/d_a 描边(有推流不影响缓存中读最新的巡线缓存)
        while not getattr(self, "_stop_flag", False):
            try:
                cap = self.cap_front
                ready = getattr(cap, "frame_ready", None)
                if ready is None:
                    time.sleep(1 / 60.0)
                    raw = cap.read()
                    show = raw.copy() if raw is not None else None
                else:
                    ready.wait()
                    ready.clear()
                    raw = cap.frame
                    show = raw.copy() if raw is not None else None
                if show is None:
                    continue
                error, angle = self._get_lane_cache()
                self._draw_lane_overlay(show, error, angle)
                self.streamer.update_frame(show, "cam1")
            except Exception as e:
                logger.warning(f"前视流转发异常: {e}")
        RealtimeMixin._front_stream_flag = False
