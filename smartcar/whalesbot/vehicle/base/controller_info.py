#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""控制器信息与协议实现: CotrollerInfo 基类 + MC601 / MC602 / MC602Wireness。

从原 serial_wrap.py 拆出, 保持对外行为与属性完全一致。
依赖 serial_protocol 的帧编解码工具与 pydownload(固件下载)。
"""
import sys
import os
import time

# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from .serial_protocol import MC_HEADER, MC_TAIL, pack_mc_frame, parse_mc_stream
from smartcar.whalesbot.vehicle.base.pydownload import Scratch_Download_MC602P

# 导入自定义log模块
from ...tools import logger


class CotrollerInfo:
    def __init__(self, baudrate, timeout=0.1, mode="USB") -> None:
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect_mode = mode
        self.name: str = None

    def send_cmd(self, cmd):
        pass

    def get_anwser(self, cmd):
        pass

    def pack_frame(self, cmd):
        """将内部命令字节打包为线路帧, 由异步引擎写盘用。默认等于命令字节本身。"""
        return cmd

    def parse_stream(self, rx, rx_start, rx_end):
        """从读线程累积的字节流(rx_start..rx_end)中切出完整应答帧。
        返回 (完整帧 bytes 或 None, 新的 rx_start)。无法成帧时返回 (None, rx_start)。
        """
        return None, rx_start

    def ping_rx(self):
        pass

    def download_bin(self, obj):
        pass

    def __str__(self) -> str:
        return "baudrate:{},timeout:{},mode:{}".format(self.baudrate, self.timeout, self.connect_mode)


class MC601(CotrollerInfo):
    def __init__(self, baudrate=380400, timeout=0.1, mode="USB") -> None:
        super().__init__(baudrate, timeout, mode)
        self.name = "mc601"
        self.header = bytes.fromhex('77 68')
        self.tail = bytes.fromhex('0A')

    def send_cmd(self, serial_obj, cmd: bytes):
        # cmd_len = len(cmd).to_bytes(1, 'big')
        # # 加入头尾数据帧
        # cmd_all = self.header + cmd_len + cmd + self.tail
        # serial_obj.write(cmd_all)
        serial_obj.write(cmd)

    def pack_frame(self, cmd):
        # MC601 发送帧: 77 68 <len> <cmd> 0A, len = len(cmd)+4(与 MC602 相同)
        return pack_mc_frame(cmd)

    def parse_stream(self, rx, rx_start, rx_end):
        # MC601 应答帧: 帧头 0x77 0x68, 第三字节为 len, 总帧长 = len+7(含 3 字节头 + 4 字节尾)
        n = len(rx)
        start = rx_start
        if n - start < 3:
            return None, start
        if rx[start] != MC_HEADER[0] or rx[start + 1] != MC_HEADER[1]:
            return None, start + 1
        total = rx[start + 2] + 7
        if n - start < total:
            return None, start
        frame = rx[start:start + total]
        if frame[-1] != MC_TAIL[0]:
            return None, start + 1
        return frame, start + total

    def get_anwser(self, serial_obj, time_out=0.05):
        time_start = time.time()
        dst_len = 0
        res = serial_obj.read(3)
        if len(res) != 3:
            return None
        # 总帧长
        dst_len = res[2] + 7
        # 获取剩余数据
        res = res + serial_obj.read(dst_len-3)
        # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
        while True:
            if time.time() - time_start > time_out:
                return None
            # data = res[3:-1]

            if len(res) == dst_len:
                if res[0] == self.header[0] and res[-1] == self.tail[0]:
                    # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
                    return res
                else:
                    return None
            res = res + serial_obj.read(dst_len - len(res))

    def ping_rx(self, serial_obj, time_out=0.05):
        time_start = time.time()
        while time.time() - time_start < time_out:
            serial_obj.reset_buffer()
            self.send_cmd(serial_obj, bytes.fromhex('77 68 04 00 01 CA 01 0A'))
            res = self.get_anwser(serial_obj, 0.03)
            if res is not None:
                # 关闭mc601省电模式
                self.send_cmd(serial_obj, bytes.fromhex('77 68 03 00 02 67 0A'))
                return True


class MC602(CotrollerInfo):
    def __init__(self, baudrate=1000000, timeout=0.1, mode="USB") -> None:
        super().__init__(baudrate, timeout, mode)
        self.name = "mc602"
        self.header = bytes.fromhex('77 68')
        self.tail = bytes.fromhex('0A')

    def send_cmd(self, serial_obj, cmd: bytes):
        cmd_len = (len(cmd) + 4).to_bytes(1, 'big')
        # 加入头尾数据帧
        cmd_all = self.header + cmd_len + cmd + self.tail
        serial_obj.write(cmd_all)
        # logger.info("send cmd:\'{}\'".format(cmd_all.hex(' ')))

    def pack_frame(self, cmd):
        return pack_mc_frame(cmd)

    def parse_stream(self, rx, rx_start, rx_end):
        return parse_mc_stream(rx, rx_start)

    def get_anwser(self, serial_obj, time_out=0.2):
        # time.sleep(0.1)
        # res = serial_obj.read(2)
        # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
        time_start = time.time()
        dst_len = 0
        res = serial_obj.read(3)
        if len(res) != 3:
            return None
        # 总帧长
        dst_len = res[2]
        # 获取剩余数据
        res = res + serial_obj.read(dst_len-3)
        while True:
            if time.time() - time_start > time_out:
                return None
            # data = res[3:-1]
            # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
            if len(res) == dst_len:
                if res[0] == self.header[0] and res[-1] == self.tail[0]:
                    return res[3:-1]
                else:
                    return None
            res = res + serial_obj.read(dst_len - len(res))


    def ping_rx(self, serial_obj, time_out=0.05):
        time_start = time.time()

        while time.time() - time_start < time_out:
            serial_obj.reset_buffer()
            self.send_cmd(serial_obj, bytes.fromhex('02 01 10'))
            res = self.get_anwser(serial_obj, 0.02)
            if res is not None:
                return True
        return False

    def download_bin(self, serial_obj):
        is_mc602 = False
        # 独占串口阶段: 暂停读线程, 防止状态机吃到下载帧
        serial_obj.pause_rx()
        try:
            return self._download_bin_impl(serial_obj)
        finally:
            serial_obj.resume_rx()

    def _download_bin_impl(self, serial_obj):
        is_mc602 = False
        serial_obj.write(bytes.fromhex('55 AA 00 01 08 00 00 F7'))
        time.sleep(0.01)
        ret = serial_obj.read(10)
        # print(ret.hex())
        if ret == bytes.fromhex('66 BB 01 01 0A 00 5A 02 00 76'):
            is_mc602 = True
            logger.info("is mc602")
            logger.info("load program")
            # 启动控制器加载程序
            start_time = time.time()
            while time.time() - start_time < 1:
                serial_obj.reset_buffer()
                serial_obj.write(bytes.fromhex('55 AA 00 40 0B 00 00 D0 00 08 DD'))
                time.sleep(0.01)
                ret = serial_obj.read(11)
                if ret == bytes.fromhex("66 BB 01 41 0B 00 00 D0 00 08 B9"):
                    break
            if self.ping_rx(serial_obj, 2):
                return True

        if is_mc602:
            # 下载程序并进入program程序
            logger.info("downloading program")
            serial_obj.close()
            result, msg = Scratch_Download_MC602P("RunA", isrun=True)

            serial_obj.open()
            if self.ping_rx(serial_obj, time_out=1.5):
                return True
        return False


class MC602Wireness(CotrollerInfo):
    def __init__(self, baudrate=115200, timeout=0.2, mode="Wireness") -> None:
        super().__init__(baudrate, timeout, mode)
        self.name = "mc602_wireness"
        self.header = bytes.fromhex('FE')
        self.header_escape = bytes.fromhex('FE FC')
        self.tail = bytes.fromhex('FF')
        self.tail_escape = bytes.fromhex('FE FD')
        self.port_src = bytes.fromhex('90')
        self.port_dst = bytes.fromhex('91')
        self.target_id = bytes.fromhex('5D 3D')

    def set_target_id(self, target_id: bytes):
        self.target_id = target_id

    def send_cmd(self, serial_obj, cmd: bytes):
        cmd_len = (len(cmd) + 4).to_bytes(1, 'big')
        # 端口地址数据组合
        cmd_data = self.port_src + self.port_dst + self.target_id + cmd
        # 转义处理
        cmd_data_escape = cmd_data.replace(self.header, self.header_escape).replace(self.tail, self.tail_escape)
        # 加入头尾数据帧
        cmd_all = self.header + cmd_len + cmd_data_escape + self.tail
        serial_obj.write(cmd_all)
        # logger.info("send cmd:\'{}\'".format(cmd_all.hex(' ')))

    def get_anwser(self, serial_obj, time_out=0.15):
        # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
        time_start = time.time()
        res = b''
        while True:
            if time.time() - time_start > time_out:
                logger.error("get_anwser timeout {}".format(res.hex(' ')))
                return None
            res = serial_obj.read(2)
            if len(res) == 2:
                break
        dst_len = res[1] + 3
        res = res + serial_obj.read(dst_len - 2)
        # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
        while True:
            if time.time() - time_start > time_out:
                return None
            # logger.info("get_anwser:\'{}\'".format(res.hex(' ')))
            res = res.replace(self.header_escape, self.header).replace(self.tail_escape, self.tail)
            rx_len = len(res)
            if rx_len == dst_len:
                if res[0] == self.header[0] and res[-1] == self.tail[0]:
                    return res[6:-1]
            res = res + serial_obj.read(dst_len - len(res))

    def ping_rx(self, serial_obj, time_out=0.3):
        self.send_cmd(serial_obj, bytes.fromhex('02 01 10'))
        # serial_obj.flush()   # 直到发送完毕
        # time.sleep(0.01)
        ret = self.get_anwser(serial_obj, time_out)
        if ret is not None:
            return True
        return False
