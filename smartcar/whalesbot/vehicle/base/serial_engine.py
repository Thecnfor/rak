#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""异步串口收发引擎: AsyncSerialEngine。

从原 serial_wrap.py 拆出, 行为与原实现完全一致:
  - 常驻读线程: select 等待可读 -> 累积字节 -> 按当前控制器的 parse_stream 切帧
    -> 按 pending seq 唤醒同步调用方 / 触发事件回调。
  - 发送: submit() 组帧后直接带锁 write(单 fd 时序天然串行), 不回包等待。
  - 兼容: get_anwser(cmd, time_out) = submit + 等 Event, 语义与旧实现一致。

所有共享状态(_pending/_callbacks)由 _lock 保护; 读线程为 daemon。
"""
import sys
import os
import select
import time
from collections import defaultdict
from threading import Event, Lock, Thread

# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from .serial_protocol import parse_mc_stream
from ...tools import logger


class AsyncSerialEngine:
    """单串口异步收发引擎。"""

    def __init__(self, ser):
        self.ser = ser
        self._lock = Lock()
        self._pending = {}  # seq -> [Event, result, matched_frame, callback]
        self._callbacks = defaultdict(list)  # (dev_id, mode, port) -> [callback]
        self._seq = 0
        self._rx_buf = bytearray()
        self._rx_start = 0
        self._thread = None
        self._closed = False
        self._paused = False  # 固件下载等独占阶段暂停读线程
        self._pause_evt = None

    # ---------- 生命周期 ----------
    def start(self):
        if self._thread is not None:
            return
        self._closed = False
        self._thread = Thread(target=self._rx_loop, name="serial_rx", daemon=True)
        self._thread.start()
        logger.info("异步串口读线程已启动")

    def close(self):
        self._closed = True
        t = self._thread
        self._thread = None
        if t is not None:
            t.join(timeout=1.0)
        # 唤醒所有等待者, 避免卡死
        with self._lock:
            for seq, item in list(self._pending.items()):
                ev = item[0]
                if ev is not None:
                    ev.set()
            self._pending.clear()
        logger.info("异步串口引擎已关闭")

    def pause(self):
        """独占串口阶段(如下载固件)暂停读线程, 防止状态机吃下载帧。"""
        self._paused = True
        self._pause_evt = Event()
        # 等读线程感知到暂停(最多等 100ms)
        self._pause_evt.wait(timeout=0.1)

    def resume(self):
        self._paused = False
        if self._pause_evt is not None:
            self._pause_evt.set()
        self._pause_evt = None

    # ---------- 发送 ----------
    def _next_seq(self):
        with self._lock:
            self._seq += 1
            return self._seq

    def submit(self, cmd: bytes, timeout=0.2, callback=None, pending=True):
        """发送一帧并登记应答等待。返回 seq。
        总是登记(单 fd 一问一答, 每个命令都期待回包, 需消费防止串包):
          - pending=True : get_anwser 等 Event
          - pending=False: 不阻塞, 回包到达时若有 callback 则回调, 随后移除登记
        """
        seq = self._next_seq()
        frame = self.ser.dev.pack_frame(cmd)
        ev = Event() if pending else None
        with self._lock:
            self._pending[seq] = [ev, None, None, callback]
        # 发送(带写锁, 单 fd 天然串行)
        with self.ser.lock:
            self.ser.write(frame)
            self.ser.flush()
        return seq

    def send_raw(self, data: bytes):
        """直写原始字节(不带帧头尾, 供 MC601 直写路径/蜂鸣器等), 不等待应答。"""
        with self.ser.lock:
            self.ser.write(data)
            self.ser.flush()

    def get_anwser(self, cmd: bytes, time_out=0.2) -> bytes:
        """同步兼容入口: 提交并等待应答。语义与旧 SerialWrap.get_anwser 一致。"""
        seq = self.submit(cmd, timeout=time_out)
        item = None
        with self._lock:
            item = self._pending.get(seq)
        if item is None:
            return None
        ev = item[0]
        if not ev.wait(time_out):
            # 超时: 移除登记, 返回 None
            with self._lock:
                if seq in self._pending:
                    del self._pending[seq]
            return None
        with self._lock:
            if seq in self._pending:
                result = self._pending[seq][1]
                del self._pending[seq]
                return result
        return None

    # ---------- 事件订阅 ----------
    def subscribe(self, dev_id, mode, port, callback):
        """订阅某(dev_id, mode, port)的应答/上报帧, 收到即回调。"""
        with self._lock:
            self._callbacks[(dev_id, mode, port)].append(callback)

    def unsubscribe(self, dev_id, mode, port, callback):
        with self._lock:
            cbs = self._callbacks.get((dev_id, mode, port))
            if cbs and callback in cbs:
                cbs.remove(callback)

    # ---------- 读线程 ----------
    def _rx_loop(self):
        while not self._closed:
            if self._paused:
                # 独占阶段: 简单轮询等待恢复
                time.sleep(0.01)
                continue
            try:
                r, _, _ = select.select([self.ser], [], [], 0.05)
                if not r:
                    continue
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:
                logger.error("串口读线程异常: {}".format(e))
                time.sleep(0.01)
                continue
            if not data:
                continue
            self._rx_buf.extend(data)
            self._dispatch()

    def _dispatch(self):
        """从 rx 缓冲切出所有完整帧并分发。"""
        dev = getattr(self.ser, "dev", None)
        parse = (
            dev.parse_stream
            if (dev is not None and hasattr(dev, "parse_stream"))
            else parse_mc_stream
        )
        while True:
            payload, start = parse(
                bytes(self._rx_buf), self._rx_start, len(self._rx_buf)
            )
            if payload is None:
                if start > self._rx_start:
                    # 跳过了脏字节, 收缩缓冲
                    del self._rx_buf[:start]
                    self._rx_start = 0
                break
            # 消费掉已切出的帧
            del self._rx_buf[:start]
            self._rx_start = 0
            self._handle_frame(payload)

    def _handle_frame(self, payload: bytes):
        """按应答/上报分发。payload 形如 <dev_id> <mode> <port> <data...>。"""
        if len(payload) < 3:
            logger.warning("收到过短帧: {}".format(payload.hex(" ")))
            return
        dev_id, mode, port = payload[0], payload[1], payload[2]
        # 1) 优先唤醒 pending(先到先得, 匹配最早等待者; 单 fd 一问一答天然 FIFO)
        with self._lock:
            seq_matched = None
            for seq, item in self._pending.items():
                seq_matched = seq
                break
            if seq_matched is not None:
                item = self._pending[seq_matched]
                item[1] = payload
                if item[0] is not None:
                    # 同步项: 唤醒 get_anwser(由 get_anwser 移除登记)
                    item[0].set()
                else:
                    # 异步项: 回包已消费, 触发回调后立即移除, 防止僵尸项卡住后续同步等待
                    cb = item[3]
                    if cb is not None:
                        try:
                            cb(payload)
                        except Exception as e:
                            logger.error("回调异常: {}".format(e))
                    del self._pending[seq_matched]
        # 2) 触发订阅回调
        with self._lock:
            cbs = list(self._callbacks.get((dev_id, mode, port), []))
        for cb in cbs:
            try:
                cb(payload)
            except Exception as e:
                logger.error("回调异常: {}".format(e))
