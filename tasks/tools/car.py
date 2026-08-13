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

from .helpers import sellect_program
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
        self.path_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
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
        time.sleep(0.2)


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
        # lane_pid_cfg = cfg['lane_pid']
        # self.pid_y = PID(lane_pid_cfg['y'], 0, 0)
        # self.lane_pid = LanePidCal(**cfg['lane_pid'])
        # self.det_pid = DetPidCal(**cfg['det_pid'])
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


    @staticmethod
    def get_cfg(path):
        """
        获取配置文件

        读取并解析YAML配置文件，将端口号转换为整数类型。

        参数:
            path: 配置文件路径
        """
        from yaml import load, Loader

        # 把配置文件读取到内存
        with open(path, "r") as stream:
            yaml_dict = load(stream, Loader=Loader)
        port_list = yaml_dict["port_io"]
        # 转化为int
        for port in port_list:
            port["port"] = int(port["port"])
        # print(yaml_dict)


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


    @staticmethod
    def get_list_by_val(list, index, val):
        """
        根据某个值获取列表中匹配的结果

        参数:
            list: 要搜索的列表
            index: 要匹配的索引位置
            val: 要匹配的值

        返回:
            匹配的元素，如果没有匹配的则返回None
        """
        for det in list:
            if det[index] == val:
                return det
        return None


    def debug(self, inference=False):
        """
        调试方法,显示摄像头图像和检测结果，用于调试和测试。

        inference: 是否进行推理，默认为False
        """
        inference_flag = False
        grasp_flag = False
        while True:
            if self._stop_flag:
                return

            keys_val = self.blue_pad.read()

            # ==================== 1. 蓝牙手柄连接检测 ====================
            if keys_val == [-1, -1, -1, -1, 0]:
                self.car_state = [0.0, 0.0, 0.0]
                logger.error("未检测到蓝牙手柄")
                self.display.show("can't find bluetooth pad\n")
                self.beep()
                time.sleep(1)
                continue

            if inference_flag:  # 按键1: 显示车道检测结果
                self.get_lane_results()
                self.get_detection_results()
            else:
                self.streamer.update_frame(self.cap_front.read(), "cam1")
                self.streamer.update_frame(self.cap_side.read(), "cam2")

            # 执行车辆控制
            self.set_velocity(keys_val[1], -keys_val[0], -keys_val[2])

            # 射击 按下【4】
            if keys_val[4] == (1 << 11):
                self.shooting()

            if keys_val[4] == (1 << 14):  # 按键[1]: 切换推理显示
                inference_flag = not inference_flag
                self.beep()
                time.sleep(0.5)

            # 执行机械臂控制
            if keys_val[4] == (1 << 4):  # 按键△ : 向上移动机械臂
                self.arm.motor_y.set_velocity(0.5)
            elif keys_val[4] == (1 << 6):  # 按键▽: 向下移动机械臂
                self.arm.motor_y.set_velocity(-0.5)
            else:
                self.arm.motor_y.set_velocity(0.0)

            if keys_val[4] == (1 << 7):  # 按键◁ : 向左移动机械臂
                self.arm.motor_x.set_angular(50)
            elif keys_val[4] == (1 << 5):  # 按键▷: 向右移动机械臂
                self.arm.motor_x.set_angular(-50)
            else:
                self.arm.motor_x.set_angular(0.0)

            if keys_val[4] == (1 << 0):  # 按键^ : 控制手臂向上<>^v
                self.arm.set_hand_angle("UP")
            elif keys_val[4] == (1 << 2):  # 按键V: 控制手臂向下<>^v
                self.arm.set_hand_angle("DOWN")

            if keys_val[4] == (1 << 1):
                self.arm.set_arm_angle("LEFT")
            elif keys_val[4] == (1 << 3):
                self.arm.set_arm_angle("RIGHT")
            elif keys_val[4] == (1 << 10):
                self.arm.set_arm_angle(-110)
                self.arm.set_hand_angle(30)

            if keys_val[4] == (1 << 9):
                grasp_flag = not grasp_flag
                self.arm.grasp(grasp_flag)
                time.sleep(0.3)
            if keys_val[4] == (1 << 8):
                self.servo_1_flag = (self.servo_1_flag + 1) % 2
                angle = self.servo_1_angle_list[self.servo_1_flag]
                print(angle)
                self.servo_1.set_angle(angle)
                time.sleep(0.3)
            time.sleep(0.05)


    def walk_lane_test(self):
        """
        车道行走测试

        测试车道保持功能，以固定速度行驶。
        """

        def end_function():
            return True

        self.lane_base(0.3, end_function, stop=self.STOP_PARAM)


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


    def manage(self, programs_list: list, order_index=0):
        """
        程序管理方法

        管理和执行程序列表，通过按键选择要执行的程序。

        参数:
            programs_list: 程序列表，包含要执行的函数
            order_index: 初始选中的程序索引，默认为0
        """

        def all_task():
            time.sleep(4)
            for func in programs_list:
                func()

        def lane_test():
            self.lane_dis_offset(0.3, 30)

        programs_suffix = [all_task, lane_test, self.debug]
        programs = programs_list.copy()
        programs.extend(programs_suffix)
        # print(programs)
        # 选中的python脚本序号
        # 当前选中的序号
        win_num = 5
        win_order = 0
        # 把programs的函数名转字符串
        logger.info(order_index)
        programs_str = [str(i.__name__) for i in programs]
        logger.info(programs_str)
        dis_str = sellect_program(programs_str, order_index, win_order)
        self.display.show(dis_str)

        self.stop()
        run_flag = False
        stop_flag = False
        stop_count = 0
        while True:
            # self.button_all.event()
            btn = self.key.get_key()
            # 短按1=1,2=2,3=3,4=4
            # 长按1=5,2=6,3=7,4=8
            # logger.info(btn)
            # button_num = car.button_all.clicked()

            if btn != 0:
                # logger.info(btn)
                # 长按1按键，退出
                if btn == 5:
                    # run_flag = True
                    self._stop_flag = True
                    self._end_flag = True
                    break
                else:
                    if btn == 4:
                        # 序号减1
                        self.beep()
                        if order_index == 0:
                            order_index = len(programs) - 1
                            win_order = win_num - 1
                        else:
                            order_index -= 1
                            if win_order > 0:
                                win_order -= 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 2:
                        self.beep()
                        # 序号加1
                        if order_index == len(programs) - 1:
                            order_index = 0
                            win_order = 0
                        else:
                            order_index += 1
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num - 1:
                                win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 3:
                        # 确定执行
                        # 调用别的程序
                        dis_str = "\n{} running......\n".format(
                            str(programs_str[order_index])
                        )
                        self.display.show(dis_str)
                        self.beep()
                        self._stop_flag = False
                        programs[order_index]()
                        self._stop_flag = True
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.stop()
                        self.beep()

                        # 自动跳转下一条
                        # if order_index == len(programs)-1:
                        #     order_index = 0
                        #     win_order = 0
                        # else:
                        #     order_index += 1
                        #     if len(programs) < win_num:
                        #         win_num = len(programs)
                        #     if win_order != win_num-1:
                        #         win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)
                    logger.info(programs_str[order_index])
            else:
                self.delay(0.02)

            time.sleep(0.02)

        for i in range(2):
            self.beep()
            time.sleep(0.4)
        time.sleep(0.1)
        self.close()



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
