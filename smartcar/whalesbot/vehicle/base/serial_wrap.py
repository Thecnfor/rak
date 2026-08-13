#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import os
from serial.tools import list_ports
from threading import Lock, Thread
from typing import List
import serial
import time
import sys
# print(time.time())
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 

from smartcar.whalesbot.vehicle.base.pydownload import Scratch_Download_MC602P

# 导入自定义log模块
from ...tools import logger
# logger.info("start time:{}".format(time.time()))

class CotrollerInfo:
    def __init__(self, baudrate, timeout=0.1, mode="USB") -> None:
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect_mode = mode
        self.name:str = None

    def send_cmd(self, cmd):
        pass

    def get_anwser(self, cmd):
        pass
    
    def ping_rx(self):
        pass
    
    def download_bin(self, obj):
        pass

    def __str__(self) -> str:
        return "baudrate:{},timeout:{},mode:{}".format(self.baudrate, self.timeout, self.connect_mode)

class SerialWrap(serial.Serial):
    def __init__(self):
        super(SerialWrap, self).__init__(port=None, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, \
                                         stopbits=serial.STOPBITS_ONE, timeout=0.03, xonxoff=False, rtscts=False, \
                                         dsrdtr=False)
        mc601 = MC601()
        mc602_usb = MC602()
        mc602_wireness = MC602Wireness()
        self.dev_list:List[CotrollerInfo] = [mc601, mc602_usb, mc602_wireness]
        self.dev = None
        self.connect_flag = False

        self.lock = Lock()
        self.timeout = 0.01
        while True:
            self.dev:CotrollerInfo =  self.ping_port()
            if self.dev is not None:
                logger.info("port is {}, controller is {}, mode {}".format(self.port, self.dev.name, self.dev.connect_mode))
                break
            logger.critical("未接控制器或者控制器没有开机,或者程序运行错误!")
            while True:
                time.sleep(1)
        self.timeout = 0.1
        
    def get_anwser(self, cmd:bytes, time_out=0.1)->bytes:
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
        port_list = [port for port in port_list if "CH340" in port[1] or "USB" in port[1]]
        port_list.sort(key=lambda x: "CH340" not in x[1])
        return port_list
    
    def set_ctl_serial(self, ctl_dev:CotrollerInfo):
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

    def assert_dev(self, name_test:str):
        # 转成小写对比
        name_dev = self.dev.name.lower()
        name_test = name_test.lower()
        if name_test in name_dev or name_dev in name_test:
            return True
        else:
            logger.error(f"dev is not {name_test}")
            while True:
                time.sleep(1)

class MC601(CotrollerInfo):
    def __init__(self, baudrate=380400, timeout=0.1, mode="USB") -> None:
        super().__init__(baudrate, timeout, mode)
        self.name = "mc601"
        self.header = bytes.fromhex('77 68')
        self.tail = bytes.fromhex('0A')

    def send_cmd(self, serial_obj:SerialWrap, cmd:bytes):
        # cmd_len = len(cmd).to_bytes(1, 'big')
        # # 加入头尾数据帧
        # cmd_all = self.header + cmd_len + cmd + self.tail
        # serial_obj.write(cmd_all)
        serial_obj.write(cmd)

    def get_anwser(self, serial_obj:SerialWrap, time_out=0.05):
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
    
    def ping_rx(self, serial_obj:SerialWrap, time_out=0.05):
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

    def send_cmd(self, serial_obj:SerialWrap, cmd:bytes):
        cmd_len = (len(cmd) + 4).to_bytes(1, 'big')
        # 加入头尾数据帧
        cmd_all = self.header + cmd_len + cmd + self.tail
        serial_obj.write(cmd_all)
        # logger.info("send cmd:\'{}\'".format(cmd_all.hex(' ')))

    def get_anwser(self, serial_obj:SerialWrap, time_out=0.2):
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

    
    def ping_rx(self, serial_obj:SerialWrap, time_out=0.05):
        time_start = time.time()

        while time.time() - time_start < time_out:
            serial_obj.reset_buffer()
            self.send_cmd(serial_obj, bytes.fromhex('02 01 10'))
            res = self.get_anwser(serial_obj, 0.02)
            if res is not None:
                return True
        return False

    def download_bin(self, serial_obj:SerialWrap):
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

    def set_target_id(self, target_id:bytes):
        self.target_id = target_id

    def send_cmd(self, serial_obj:SerialWrap, cmd:bytes):
        cmd_len = (len(cmd) + 4).to_bytes(1, 'big')
        # 端口地址数据组合
        cmd_data = self.port_src + self.port_dst + self.target_id + cmd
        # 转义处理
        cmd_data_escape = cmd_data.replace(self.header, self.header_escape).replace(self.tail, self.tail_escape)
        # 加入头尾数据帧
        cmd_all = self.header + cmd_len + cmd_data_escape + self.tail
        serial_obj.write(cmd_all)
        # logger.info("send cmd:\'{}\'".format(cmd_all.hex(' ')))

    def get_anwser(self, serial_obj:SerialWrap, time_out=0.15):
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
    
    def ping_rx(self, serial_obj:SerialWrap, time_out=0.3):
        self.send_cmd(serial_obj, bytes.fromhex('02 01 10'))
        # serial_obj.flush()   # 直到发送完毕
        # time.sleep(0.01)
        ret = self.get_anwser(serial_obj, time_out)
        if ret is not None:
            return True
        return False

serial_wrap = SerialWrap()
# logger.info("start time:{}".format(time.time()))

if __name__ == "__main__":
    last_time = time.time()
    # print(time.time())
    serial_wrap.timeout=0.3
    while True:
        # serial_wra
        serial_wrap.reset_buffer()
        ret = serial_wrap.get_anwser(bytes.fromhex('02 02 01 10'))
        # print(ret)
        time.sleep(0.4)
        # serial_wrap.write(bytes.fromhex('FE 10 90 91 5d 3d 02 02 01 10 02 02 02 10 02 02 03 10 FF'))
        # res = serial_wrap.read(23)
        # if(res != b'\xfe\x14\x90\x91]=\x02\x02\x01\x10\x02\x02\x02\x10\x02\x02\x03\x10\x02\x02\x04\n\xff'):
        #     print(res)
        # ret = serial_wrap.get_anwser(bytes.fromhex('02 02 01 F0'))
        # if ret is not None:
        #     logger.info("ret:\'{}\'".format(ret.hex(' ')))

        # fps = 1.0 / (time.time() - last_time)
        # last_time = time.time()
        # logger.info("fps:{}".format(fps))
        # time.sleep(1)
    # logger.info()