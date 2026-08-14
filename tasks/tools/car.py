#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
import threading
import time

from smartcar import Camera, Streamer, logger
from smartcar.whalesbot.tools import get_yaml
from smartcar.whalesbot.vehicle import (
    ArmController,
    Beep,
    BluetoothPad,
    Key4Btn,
    MecanumDriver,
    ScreenShow,
    ServoPwm,
)
from smartcar.whalesbot.vehicle.base.controller_wrap import PoutD

from .motion import MotionMixin
from .perception import PerceptionMixin
from .pids import PidCal2

# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__)))


class MyCar(MotionMixin, PerceptionMixin, MecanumDriver):
    """
    智能车控制类

    该类继承自MecanumDriver，实现了智能车的完整控制功能，包括传感器初始化、PID控制、摄像头控制、
    目标检测、车道保持等功能。移动与感知方法分别由 MotionMixin / PerceptionMixin 提供。
    """

    STOP_PARAM: bool = True

    def __init__(self):
        """
        初始化智能车

        初始化智能车的各个组件，包括底盘、传感器、摄像头、PID控制器等。
        """
        # 调用继承的初始化
        start_time = time.time()
        super(MyCar, self).__init__()
        logger.info("my car init ok {}".format(time.time() - start_time))
        # 显示
        self.display = ScreenShow()

        self.streamer = Streamer()
        self.arm = ArmController()

        # 获取自己文件所在的目录路径
        self.path_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..")
        )
        self.yaml_path = os.path.join(self.path_dir, "config_car.yml")
        # 获取配置
        cfg = get_yaml(self.yaml_path)
        # 根据配置设置sensor
        self.sensor_init(cfg)

        self.car_pid_init(cfg)
        self.ring = Beep()
        self.camera_init(cfg)
        # paddle推理初始化
        self.paddle_infer_init()
        # 侧视实时流(cam2)转发 + 持续检测线程(后端就绪后再启动, 避免空等)
        self.start_side_stream()
        # 文心一言分析初始化
        self.ernie_bot_init()

        # 相关临时变量设置
        # 程序结束标志
        self._stop_flag = False
        # 按键线程结束标志
        self._end_flag = False
        self.thread_key = threading.Thread(target=self.key_thread_func)
        self.thread_key.daemon = True
        self.thread_key.start()

        self.beep()

    def beep(self):
        """
        发出蜂鸣音

        控制蜂鸣器发出一声蜂鸣音，并等待0.2秒。
        """
        self.ring.rings()

    def sensor_init(self, cfg):
        """
        初始化传感器

        根据配置初始化按键、灯光和红外传感器。

        参数:
            cfg: 配置字典，包含传感器的配置信息

        """
        cfg_sensor = cfg["io"]
        # print(cfg_sensor)
        self.key = Key4Btn(cfg_sensor["key"])
        # self.light = LedLight(cfg_sensor['light'])
        # self.left_sensor = Infrared(cfg_sensor['left_sensor'])
        # self.right_sensor = Infrared(cfg_sensor['right_sensor'])
        self.servo_1_angle_list = [-42, 165]
        self.servo_1_flag = 0
        self.servo_1 = ServoPwm(1, 180)
        self.servo_1.set_angle(self.servo_1_angle_list[self.servo_1_flag])
        self.blue_pad = BluetoothPad()
        self.shoot = PoutD(4)

    def set_storage(self, state=False):
        """
        设置储存仓的位置

        根据状态参数控制储存仓的开关。

        参数:
            state (bool): 储存仓状态。False 表示放下，True 表示收起。默认为 False。
        """
        flag = 1 if state else 0
        self.servo_1.set_angle(self.servo_1_angle_list[flag])

    def shooting(self):
        self.shoot.set(1)
        time.sleep(0.3)
        self.shoot.set(0)
        time.sleep(0.5)

    def car_pid_init(self, cfg):
        """
        初始化PID控制器

        根据配置初始化车道保持和目标检测的PID控制器。

        参数:
            cfg: 配置字典，包含PID控制器的配置信息
        """
        self.lane_pid = PidCal2(**cfg["lane_pid"])
        self.det_pid = PidCal2(**cfg["det_pid"])

    def camera_init(self, cfg):
        """
        初始化摄像头

        根据配置初始化前置摄像头和侧面摄像头。

        参数:
            cfg: 配置字典，包含摄像头的配置信息
        """
        # 初始化前后摄像头设置
        self.cap_front = Camera(cfg["camera"]["front"])
        # 侧面摄像头
        self.cap_side = Camera(cfg["camera"]["side"])

    def delay(self, time_hold):
        """
        延时函数

        延时指定的时间，期间会检查停止标志。

        参数:
            time_hold: 延时时间（秒）
        """
        start_time = time.time()
        while True:
            if self._stop_flag:
                return
            if time.time() - start_time > time_hold:
                break

    def key_thread_func(self):
        """
        按键检测线程

        持续检测按键状态，当检测到按键3时设置停止标志。
        """
        while True:
            if not self._stop_flag:
                if self._end_flag:
                    return
                key_val = self.key.get_key()
                # print(key_val)
                if key_val == 3:
                    self._stop_flag = True
                time.sleep(0.2)

    def close(self):
        """
        关闭方法

        关闭所有线程和资源，包括按键线程、摄像头和流处理器。
        """
        self._stop_flag = False
        self._end_flag = True
        self.thread_key.join()
        self.cap_front.close()
        self.cap_side.close()
        self.streamer.stop()
        # self.grap_cam.close()


def create_car(reset=True):
    """Create the competition car and perform the standard preparation."""
    car = MyCar()
    car.STOP_PARAM = False
    if reset:
        car.beep()
        time.sleep(1)
        car.arm.reset_position()
        car.reset_position()
    return car
