#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""串口封装与转发层: SerialWrap + 模块级单例。

本文件为兼容转发层(对外接口保持不变), 实现已拆分为:
  - serial_protocol : 帧编解码纯函数(MC_HEADER / MC_TAIL / pack_mc_frame / parse_mc_stream)
  - serial_engine   : AsyncSerialEngine 异步引擎
  - controller_info : CotrollerInfo / MC601 / MC602 / MC602Wireness

外部 `from .serial_wrap import ...` 或 `import *` 的用法完全不变。
"""
import sys
import os
import time
from serial.tools import list_ports
from threading import RLock
from typing import List
import serial

# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

# 从拆分模块导入实现
from .serial_protocol import (
    MC_HEADER,
    MC_TAIL,
    pack_mc_frame,
    parse_mc_stream,
)
from .serial_engine import AsyncSerialEngine
from .controller_info import (
    CotrollerInfo,
    MC601,
    MC602,
    MC602Wireness,
)

# 导入自定义log模块
from ...tools import logger

# logger.info("start time:{}".format(time.time()))


class SerialWrap(serial.Serial):
    def __init__(self):
        super(SerialWrap, self).__init__(
            port=None,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.03,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        mc601 = MC601()
        mc602_usb = MC602()
        mc602_wireness = MC602Wireness()
        self.dev_list: List[CotrollerInfo] = [mc601, mc602_usb, mc602_wireness]
        self.dev = None
        self.connect_flag = False

        self.lock = RLock()
        self.timeout = 0.01
        while True:
            self.dev: CotrollerInfo = self.ping_port()
            if self.dev is not None:
                logger.info(
                    "port is {}, controller is {}, mode {}".format(
                        self.port, self.dev.name, self.dev.connect_mode
                    )
                )
                break
            logger.critical("未接控制器或者控制器没有开机,或者程序运行错误!")
            while True:
                time.sleep(1)
        self.timeout = 0.1
        # 异步引擎: 保持 get_anwser 语义不变, 底层改为事件驱动读写
        self.engine = AsyncSerialEngine(self)
        if os.environ.get("SMARTCAR_SERIAL_SYNC", "") != "1":
            self.engine.start()
            logger.info("异步串口引擎已启动")

    def get_anwser(self, cmd: bytes, time_out=0.1) -> bytes:
        # 同步兼容入口: 交由异步引擎 submit + 等应答, 语义与旧实现一致
        if getattr(self, "engine", None) is not None:
            return self.engine.get_anwser(cmd, time_out)
        # 回退路径(引擎未启用/初始化阶段): 保持旧式锁+阻塞一问一答
        self.lock.acquire()
        res = None
        try:
            self.reset_buffer()
            self.dev.send_cmd(self, cmd)
            res = self.dev.get_anwser(self)
        except Exception as e:
            logger.error("get_anwser error:{}".format(e))
        self.lock.release()
        return res

    # ---------- 异步引擎透传接口(供事件驱动/并发使用) ----------
    def send_async(self, cmd: bytes, callback=None, timeout=0.2):
        """异步发送命令, 不阻塞等待应答。callback(payload) 可选, 收到应答时回调。"""
        if getattr(self, "engine", None) is None:
            raise RuntimeError("异步引擎未启用, 请勿在初始化前使用 send_async")
        return self.engine.submit(
            cmd, timeout=timeout, callback=callback, pending=False
        )

    def send_raw(self, data: bytes):
        """直写原始字节(不带帧头尾), 供 MC601 直写路径/蜂鸣器等使用, 不等待应答。"""
        with self.lock:
            super(SerialWrap, self).write(data)
            super(SerialWrap, self).flush()

    def write(self, data):
        """覆盖 pyserial.write: 统一走带锁原始写, 使 MC601 直写路径自动并发安全。
        返回写字节数(与 pyserial 语义一致)。
        """
        with self.lock:
            return super(SerialWrap, self).write(data)

    def subscribe(self, dev_id, mode, port, callback):
        if getattr(self, "engine", None) is None:
            raise RuntimeError("异步引擎未启用, 无法订阅事件")
        self.engine.subscribe(dev_id, mode, port, callback)

    def unsubscribe(self, dev_id, mode, port, callback):
        if getattr(self, "engine", None) is not None:
            self.engine.unsubscribe(dev_id, mode, port, callback)

    def pause_rx(self):
        """独占串口阶段(如下载固件)暂停读线程。"""
        if getattr(self, "engine", None) is not None:
            self.engine.pause()

    def resume_rx(self):
        if getattr(self, "engine", None) is not None:
            self.engine.resume()

    def get_answer1(self, cmd: bytes, time_out=0.1):
        """兼容历史调用名(实际为 get_anwser), 避免 MC601 设备类 AttributeError。"""
        return self.get_anwser(cmd, time_out)

    def set_bps(self, bps):
        self.baudrate = bps

    def set_port(self, port):
        if self.connect_flag:
            self.close()
            self.connect_flag = False
        self.port = port

    def open(self):
        try:
            if self.port is None:
                return False
            self.connect_flag = True
            super(SerialWrap, self).open()
            return True
        except Exception as e:
            self.connect_flag = False
            return False

    def get_serial_list(self):
        port_list = list_ports.comports()
        # for port in port_list:
        #     print('端口号：' + port[0] + '   端口名：' + port[1])
        port_list = [
            port for port in port_list if "CH340" in port[1] or "USB" in port[1]
        ]
        port_list.sort(key=lambda x: "CH340" not in x[1])
        return port_list

    def set_ctl_serial(self, ctl_dev: CotrollerInfo):
        self.baudrate = ctl_dev.baudrate

    def ping_port(self):
        serial_list = self.get_serial_list()
        if len(serial_list) == 0:
            logger.error("未找到串口,查看是否插入了串口,或者查看下位机是否开机")
        while len(serial_list) == 0:
            # logger.error("未找到串口,查看是否插入了串口,或者查看下位机是否开机")
            time.sleep(1)
            serial_list = self.get_serial_list()
        for serial in serial_list:
            try:
                logger.info("try:{}".format(serial))
                self.set_port(serial[0])
                time.sleep(0.01)
                self.open()
                for ctl_dev in self.dev_list:
                    # logger.info("ping:{}".format(ctl_dev.name))
                    self.set_ctl_serial(ctl_dev)
                    if ctl_dev.ping_rx(self):
                        # logger.info(ctl_dev)
                        return ctl_dev
                for ctl_dev in self.dev_list:
                    # logger.info("try downlaod bin:{}".format(ctl_dev.name))
                    self.set_ctl_serial(ctl_dev)
                    if ctl_dev.download_bin(self):
                        return ctl_dev
                self.close()
            except Exception as e:
                logger.error(e)
        logger.error("未找到支持的设备")
        return None

    def reset_buffer(self):
        self.reset_input_buffer()
        self.reset_output_buffer()

    def assert_dev(self, name_test: str):
        # 转成小写对比
        name_dev = self.dev.name.lower()
        name_test = name_test.lower()
        if name_test in name_dev or name_dev in name_test:
            return True
        else:
            logger.error(f"dev is not {name_test}")
            while True:
                time.sleep(1)


# 模块级单例(与原实现一致, 初始化即探测串口)
serial_wrap = SerialWrap()
# logger.info("start time:{}".format(time.time()))


if __name__ == "__main__":
    last_time = time.time()
    # print(time.time())
    serial_wrap.timeout = 0.3
    while True:
        # serial_wra
        serial_wrap.reset_buffer()
        ret = serial_wrap.get_anwser(bytes.fromhex("02 02 01 10"))
        # print(ret)
        time.sleep(0.4)
