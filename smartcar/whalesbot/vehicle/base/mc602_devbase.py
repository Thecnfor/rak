#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""MC602 设备基础层: 设备注册表 / 数据编解码 / 命令接口基类。

从原 mc602_ctl2.py 拆出, 行为与原实现完全一致。
"""
import os
import sys
import struct

# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__)))
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from ...tools import logger
from smartcar.whalesbot.vehicle.base.serial_wrap import serial_wrap

serial_mc602 = serial_wrap
# def set_serial_mc602(ser:SerialWrap):
#     global serial_mc602
#     serial_mc602 = ser

ctl602_dev_list = {
    "motor4": {"dev_id": 0x01, "format": "bbbbb"},
    "motor": {"dev_id": 0x02, "format": "bbb"},
    "encoder4": {"dev_id": 0x03, "format": "biiii"},
    "encoder": {"dev_id": 0x04, "format": "bbi"},
    "servo_pwm": {"dev_id": 0x05, "format": "bbBB"},
    "servo_bus": {"dev_id": 0x06, "format": "bbbbh"},
    "sensor_analog": {"dev_id": 0x07, "mode": 0, "format": "bbH"},
    "sensor_infrared": {"dev_id": 0x07, "mode": 1, "format": "bbH"},
    "sensor_touch": {"dev_id": 0x07, "mode": 2, "format": "bbH"},
    "sensor_ultrasonic": {"dev_id": 0x07, "mode": 3, "format": "bbH"},
    "sensor_ambient_light": {"dev_id": 0x07, "mode": 4, "format": "bbH"},
    "sensor_analog_a": {"dev_id": 0x08, "mode": 0, "format": "bbH"},
    "bluetooth": {"dev_id": 0x09, "format": "BBBBi"},
    "beep": {"dev_id": 0x0a, "format": "BBB"},
    "led_show": {"dev_id": 0x0b, "format": "b"*101},
    "power": {"dev_id": 0x0c, "format": "bi"},
    "board_key": {"dev_id": 0x0d, "format": "bbb"},
    "led_light": {"dev_id": 0x0e, "format": "bbBBBB"},
    "nixietube": {"dev_id": 0x0f, "format": "bbi"},
    "dout": {"dev_id": 0x10, "format": "bbb"},
    "stepper": {"dev_id": 0x11, "format": "bbii"}
}


class StructData():
    def __init__(self, format=None) -> None:
        if format is None:
            format = ''
        self.format = '<b' + format
        self.size = struct.calcsize(self.format)
        self.len = len(self.format)-1

    def set_format(self, format):
        self.format = '<b' + format
        self.size = struct.calcsize(self.format)
        self.len = len(self.format)-1

    def __sizeof__(self) -> int:
        return self.size

    def unpack_data(self, data, index_start):
        try:
            s = index_start
            e = index_start + self.size
            # print(data[s:e])
            re_list = list(struct.unpack(self.format, data[s:e]))
        except Exception as e:
            pass
            return []
        return re_list

    def pack_data(self, data):
        bytes_t = struct.pack(self.format, *data)
        return bytes_t

    # 定义len函数的定义
    def __len__(self):
        return self.len


class DevCmdInterface:
    def __init__(self, dev_id=None, mode=None, port_id=None, format='bb') -> None:
        global serial_mc602
        self.ser = serial_mc602
        self.data_struct = StructData(format)
        self.dev_id = dev_id
        self.mode = mode
        self.port_id = port_id

        self.time_out = 0.2
        self.last_data = None
        # 参数保存位置
        self.arg_reg = 1

    def set_time_out(self, time_out):
        self.time_out = time_out

    def set_port(self, port_id):
        self.port_id = port_id

    def get_bytes(self, *args, mode=None, port_id=None):
        # 根据参数补充所有参数
        data = []
        # print(args)
        data.append(self.dev_id)
        self.arg_reg = 3
        # 根据需要添加操作参数
        if mode is not None:
            data.append(mode)
        elif self.mode is not None:
            data.append(self.mode)
        else:
            self.arg_reg -= 1
            data.append(0)
        # 根据需要添加端口参数
        if port_id is not None:
            data.append(port_id)
        elif self.port_id is not None:
            data.append(self.port_id)
        else:
            self.arg_reg -= 1
        d_len = len(self.data_struct) - len(data)
        args_list = list(args)
        # 根据情况去除参数或者补齐参数
        while True:
            if len(args_list) > d_len:
                args_list.pop(0)
            elif len(args_list) < d_len:
                args_list.append(0)
            else:
                break
        data = data + args_list
        return self.data_struct.pack_data(data)

    def get_result(self, bytes_all, index=0):
        data = self.data_struct.unpack_data(bytes_all, index)[self.arg_reg:]
        # 如果只有一个结果
        if len(data) == 1:
            data = data[0]
        return data

    def send_get(self, bytes_tmp: bytes):
        ret = self.ser.get_anwser(bytes_tmp, self.time_out)
        if ret is not None:
            self.last_data = self.get_result(ret)
        return self.last_data

    def send_async(self, bytes_tmp: bytes, callback=None, timeout=None):
        """异步发送命令, 不阻塞等待应答。收到应答时若回调非空, 回调(解析后的结果)。"""
        if timeout is None:
            timeout = self.time_out
        self.ser.send_async(bytes_tmp, callback=callback, timeout=timeout)

    def subscribe(self, callback, mode=None, port_id=None):
        """订阅本设备的应答/上报帧事件(事件驱动)。收到帧即回调(解析后的结果)。"""
        dev_id = self.dev_id
        m = mode if mode is not None else (self.mode if self.mode is not None else 0)
        p = port_id if port_id is not None else (self.port_id if self.port_id is not None else 0)
        self.ser.subscribe(dev_id, m, p, callback)

    def unsubscribe(self, callback, mode=None, port_id=None):
        dev_id = self.dev_id
        m = mode if mode is not None else (self.mode if self.mode is not None else 0)
        p = port_id if port_id is not None else (self.port_id if self.port_id is not None else 0)
        self.ser.unsubscribe(dev_id, m, p, callback)

    def act_mode(self, *args, mode=None, port_id=None):
        data_bytes = self.get_bytes(*args, mode=mode, port_id=port_id)
        return self.send_get(data_bytes)

    def reset(self, *args, port_id=None):
        data_bytes = self.get_bytes(*args, mode=3, port_id=port_id)
        return self.send_get(data_bytes)

    # 设置操作
    def set(self, *args, port_id=None):
        # print(args)
        data_bytes = self.get_bytes(*args, mode=2, port_id=port_id)
        # print(data_bytes.hex(" "))
        return self.send_get(data_bytes)

    # 获取操作
    def get(self, *args, port_id=None):
        data_bytes = self.get_bytes(*args, mode=1, port_id=port_id)
        # print(data_bytes)
        return self.send_get(data_bytes)

    # 没有操作符号时
    def no_act(self, port_id=None):
        data_bytes = self.get_bytes(port_id=port_id)
        # print(data_bytes)
        return self.send_get(data_bytes)

    def act_default(self, *args, port_id=None):
        data_bytes = self.get_bytes(*args, port_id=port_id)
        return data_bytes


class DevListWrap:
    def __init__(self, dev_list=None) -> None:
        if dev_list is None:
            self.dev_list = []
        else:
            self.dev_list = dev_list

    def get_all(self, args, mode=1):
        bytes_all = b''
        for i in range(len(self.dev_list)):
            bytes_all += self.dev_list[i].get_bytes(args[i], mode=mode)
            # bytes_all += self.dev_list[i].act_default(args[i])
        # print(bytes_all.hex(' '))
        res = serial_mc602.get_anwser(bytes_all)
        data_ret = []
        if res is not None:
            index = 0
            for i in range(len(self.dev_list)):
                data = self.dev_list[i].get_result(res, index)
                index += self.dev_list[i].data_struct.size
                data_ret.append(data)
        else:
            return [0, 0, 0, 0]
        return data_ret

    def __getattr__(self, name):
        return getattr(self.dev_list, name)
