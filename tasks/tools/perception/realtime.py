# -*- coding: utf-8 -*-
"""侧视实时流与持续检测(RealtimeMixin): 来一帧推一帧, 背靠背推理(从 perception.py 拆分而来)。"""
import json
import threading
import time

import cv2
import zmq

from smartcar.whalesbot.tools import logger


class RealtimeMixin:

    # 侧视实时流: 始终推画面, 检测线程每 0.5s 跑一次检测, 有目标就叠框
    _side_stream_flag = False

    def start_side_stream(self):
        """启动侧视(cam2)实时检测 + 实时推流两个线程, 由 MyCar 初始化时调用。"""
        if RealtimeMixin._side_stream_flag:
            return
        RealtimeMixin._side_stream_flag = True
        self._init_realtime_cache()
        threading.Thread(
            target=self._side_detect_loop, name="side_detect", daemon=True
        ).start()
        threading.Thread(
            target=self._side_stream_loop, name="side_stream", daemon=True
        ).start()

    def _init_realtime_cache(self):
        """初始化实时检测结果缓存(检测线程来一帧推一帧)。"""
        self._det_lock = threading.Lock()
        self._det_cache = None  # (timestamp, dets)

    def get_realtime_detections(self, fresh=False, max_age=None):
        """实时获取侧视 task 检测结果。

        检测线程对最新帧背靠背推理并更新缓存; 本方法非阻塞返回最新结果。
        fresh=True 时立刻同步跑一次推理(独立连接, 不阻塞检测线程)。

        返回:
            list: [cls_id, obj_id, label, score, x_c, y_c, w, h](归一化)
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
            self._init_realtime_cache()
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
