#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""RTD(C++ 实时控制守护进程)的 Python 客户端 + 串口 IPC 引擎。

启用方式: 环境变量 SMARTCAR_RTD=1(可选 SMARTCAR_RTD_CMD_URL /
SMARTCAR_RTD_PUB_URL 覆盖默认 127.0.0.1:6010/6011)。

架构(rtd 模式, 见 cpp/rtd/README.md):
    本进程 ──ZMQ──> rtd 守护进程(独占 /dev/ttyUSB0, 100Hz 控制环)
      - MecanumDriver: set_velocity/get_odometry/move_to_position 等退化为
        ZMQ 命令(rtd 内跑同样的运动学/里程计/PID 闭环);
      - serial_wrap: 其余设备(机械臂/舵机/按键/蜂鸣器)的 MC 帧透传给 rtd
        上总线, 应答按 (dev,mode,port) 匹配后原样返回。
不设 SMARTCAR_RTD 时本模块不会被激活, 全部走原串口路径(零行为变化)。
"""

import json
import os
import threading
import time

import numpy as np
import zmq

from ...tools.log_wrap import logger


def rtd_enabled() -> bool:
    """是否启用 rtd 模式(环境变量开关, 默认关闭走原串口路径)。"""
    return os.environ.get("SMARTCAR_RTD", "") == "1"


class RtdClient:
    """rtd 守护进程的 ZMQ 客户端(命令 REQ/REP + 状态 PUB/SUB)。"""

    def __init__(
        self,
        cmd_url="tcp://127.0.0.1:6010",
        pub_url="tcp://127.0.0.1:6011",
        timeout=2.0,
    ):
        self.cmd_url = cmd_url
        self.pub_url = pub_url
        self.timeout = timeout

        self._ctx = zmq.Context()
        self._req_lock = threading.Lock()
        self._sock = self._make_req()

        # 状态缓存 + 订阅分发(PUB: state 心跳 / 透传应答)
        self._state = None
        self._state_lock = threading.Lock()
        self._sub_cbs = {}  # (dev,mode,port) -> [callback(payload_bytes)]
        self._sub_lock = threading.Lock()
        self._async_cbs = {}  # seq -> callback(payload_bytes or None)
        self._async_seq = 0

        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.connect(pub_url)
        self._sub_thread = threading.Thread(
            target=self._sub_loop, name="rtd_sub", daemon=True
        )
        self._sub_thread.start()

    # ------------------------------------------------------------------
    # 命令通道(REQ/REP)
    # ------------------------------------------------------------------
    def _make_req(self):
        sock = self._ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVTIMEO, int(self.timeout * 1000))
        sock.connect(self.cmd_url)
        return sock

    def _call(self, req: dict):
        """发一条 JSON 命令并返回应答 dict; 失败返回 None。
        REQ socket 超时后会进入不可用状态, 需重建。"""
        with self._req_lock:
            try:
                self._sock.send(json.dumps(req).encode("utf-8"))
                rep = self._sock.recv()
                return json.loads(rep)
            except zmq.ZMQError as e:
                logger.warning(f"rtd 命令失败({e}): {req.get('cmd')}")
                # 重建 socket(REQ 超时后必须 close 才能复用)
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = self._make_req()
                return None

    # ---- 运动控制(MecanumDriver 用) ----
    def vel(self, x, y, z):
        return self._call({"cmd": "vel", "v": [float(x), float(y), float(z)]})

    def goto(self, target, max_v, tol, timeout):
        return self._call(
            {
                "cmd": "goto",
                "target": [float(v) for v in target],
                "max_v": [float(v) for v in max_v],
                "tol": [float(v) for v in tol],
                "timeout": float(timeout),
            }
        )

    def stop(self):
        return self._call({"cmd": "stop"})

    def reset_odom(self, x=None, y=None, z=None, distance=None):
        req = {"cmd": "reset"}
        # 与 Python Odometry.reset 一致: 字段缺省 = 保持原值
        for k, v in (("x", x), ("y", y), ("z", z), ("distance", distance)):
            if v is not None:
                req[k] = float(v)
        return self._call(req)

    # ---- 帧透传(serial_wrap 用) ----
    def frame(self, payload: bytes, timeout_ms=200):
        """同步透传: MC 命令帧上线并等应答, 返回应答 payload bytes 或 None。"""
        rep = self._call(
            {
                "cmd": "frame",
                "payload": payload.hex(),
                "timeout_ms": int(timeout_ms),
            }
        )
        if rep is None or not rep.get("ok"):
            return None
        return bytes.fromhex(rep.get("payload", ""))

    def frame_async(self, payload: bytes, callback=None, timeout_ms=200):
        """异步透传: 立即返回 seq; 应答/超时时 callback(payload 或 None)。"""
        with self._sub_lock:
            self._async_seq += 1
            seq = self._async_seq
            if callback is not None:
                self._async_cbs[seq] = callback
        rep = self._call(
            {
                "cmd": "frame_async",
                "payload": payload.hex(),
                "seq": seq,
                "timeout_ms": int(timeout_ms),
            }
        )
        if rep is None or not rep.get("ok"):
            # 提交失败: 直接回调 None 并清理登记
            with self._sub_lock:
                cb = self._async_cbs.pop(seq, None)
            if cb is not None:
                cb(None)
            return None
        return seq

    def subscribe(self, dev_id, mode, port, callback):
        """订阅某 (dev,mode,port) 的应答/上报帧(对应 serial 引擎的 subscribe)。"""
        with self._sub_lock:
            self._sub_cbs.setdefault((dev_id, mode, port), []).append(callback)
        self._call({"cmd": "sub", "dev": dev_id, "mode": mode, "port": port})

    def unsubscribe(self, dev_id, mode, port, callback):
        with self._sub_lock:
            cbs = self._sub_cbs.get((dev_id, mode, port))
            if cbs and callback in cbs:
                cbs.remove(callback)

    # ---- 状态 ----
    def get_state(self, refresh=False):
        """最新状态 dict(rtd 以 ~50Hz 推送); refresh=True 时同步拉一次。"""
        if refresh:
            rep = self._call({"cmd": "state"})
            if rep is not None:
                with self._state_lock:
                    self._state = rep
        with self._state_lock:
            return self._state

    def get_odometry(self):
        """[x, y, theta] np.float32 数组(与 MecanumDriver.get_odometry 兼容)。"""
        st = self.get_state()
        if st is None:
            return np.zeros(3, dtype=np.float32)
        return np.array([st["x"], st["y"], st["th"]], dtype=np.float32)

    def get_distance(self):
        st = self.get_state()
        return float(st["dist"]) if st else 0.0

    # ------------------------------------------------------------------
    # PUB 订阅线程
    # ------------------------------------------------------------------
    def _sub_loop(self):
        while True:
            try:
                msg = self._sub.recv_json()
            except zmq.ZMQError as e:
                logger.error(f"rtd SUB 异常: {e}")
                time.sleep(0.1)
                continue
            except Exception as e:
                logger.error(f"rtd SUB 解析异常: {e}")
                continue
            evt = msg.get("evt")
            if evt == "state":
                with self._state_lock:
                    self._state = msg
            elif evt in ("reply", "timeout"):
                seq = msg.get("seq")
                payload = None
                if evt == "reply":
                    try:
                        payload = bytes.fromhex(msg.get("payload", ""))
                    except Exception:
                        payload = None
                with self._sub_lock:
                    cb = self._async_cbs.pop(seq, None)
                if cb is not None:
                    try:
                        cb(payload)
                    except Exception as e:
                        logger.error(f"rtd 异步回调异常: {e}")
            elif evt == "frame":
                # (dev,mode,port) 订阅帧(unsolicited/上报)
                try:
                    payload = bytes.fromhex(msg.get("payload", ""))
                except Exception:
                    continue
                key = (msg.get("dev"), msg.get("mode"), msg.get("port"))
                with self._sub_lock:
                    cbs = list(self._sub_cbs.get(key, []))
                for cb in cbs:
                    try:
                        cb(payload)
                    except Exception as e:
                        logger.error(f"rtd 订阅回调异常: {e}")


# ----------------------------------------------------------------------
# 模块级单例(进程内 serial_wrap 与 MecanumDriver 共用一个连接)
# ----------------------------------------------------------------------
_rtd_singleton = None
_rtd_once = threading.Lock()


def shared_rtd():
    """返回共享 RtdClient; 未启用 rtd 模式时返回 None。"""
    global _rtd_singleton
    if not rtd_enabled():
        return None
    if _rtd_singleton is None:
        with _rtd_once:
            if _rtd_singleton is None:
                _rtd_singleton = RtdClient(
                    cmd_url=os.environ.get(
                        "SMARTCAR_RTD_CMD_URL", "tcp://127.0.0.1:6010"
                    ),
                    pub_url=os.environ.get(
                        "SMARTCAR_RTD_PUB_URL", "tcp://127.0.0.1:6011"
                    ),
                )
    return _rtd_singleton


class RtdSerialEngine:
    """serial_wrap 的 IPC 引擎: 把 AsyncSerialEngine 的接口映射到 RtdClient。

    保持 get_anwser / submit+wait_answer / send_async / subscribe 语义,
    让 DevCmdInterface 及所有设备封装(机械臂/舵机/按键...)零改动可用。
    """

    def __init__(self, rtd: RtdClient):
        self.rtd = rtd
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._results = {}  # seq -> payload bytes(同步 submit 的结果暂存)
        self._results_lock = threading.Lock()

    # ---- SerialWrap.get_anwser 委托 ----
    def get_anwser(self, cmd: bytes, time_out=0.2) -> bytes:
        return self.rtd.frame(cmd, timeout_ms=int(time_out * 1000))

    # ---- SerialWrap.submit/wait_answer 委托(DevListWrap.get_all 用) ----
    def submit(self, cmd: bytes, timeout=0.2, callback=None, pending=True):
        if pending:
            # 同步语义: rtd 的 frame 一问一答, 结果暂存后由 wait 立即取走
            with self._seq_lock:
                self._seq += 1
                seq = self._seq
            res = self.rtd.frame(cmd, timeout_ms=int(timeout * 1000))
            with self._results_lock:
                self._results[seq] = res
            return seq
        # pending=False: 真异步, 应答走 PUB 回调
        return self.rtd.frame_async(
            cmd, callback=callback, timeout_ms=int(timeout * 1000)
        )

    def wait(self, seq, timeout=0.2):
        if seq is None:
            return None
        with self._results_lock:
            return self._results.pop(seq, None)

    def send_async(self, cmd: bytes, callback=None, timeout=0.2):
        self.submit(cmd, timeout=timeout, callback=callback, pending=False)

    # ---- 订阅(事件驱动设备用) ----
    def subscribe(self, dev_id, mode, port, callback):
        self.rtd.subscribe(dev_id, mode, port, callback)

    def unsubscribe(self, dev_id, mode, port, callback):
        self.rtd.unsubscribe(dev_id, mode, port, callback)

    # ---- 串口独占(固件下载)在 rtd 模式下不适用 ----
    def pause(self):
        logger.warning("rtd 模式不支持串口独占(pause), 已忽略")

    def resume(self):
        pass
