#!/usr/bin/python
# -*- coding: utf-8 -*-
from urllib import response
import base64
import psutil
from typing import Union
import time
import threading
import os
import platform
import signal
from smartcar import Camera, Streamer
import numpy as np
from smartcar.whalesbot.vehicle import (
    ArmController,
    ScreenShow,
    Key4Btn,
    Infrared,
    LedLight,
    MecanumDriver,
    Beep,
    BluetoothPad,
    ServoPwm,
)
from smartcar import PID
import difflib
import cv2
import math
from smartcar.paddlebaidu.infer_cs import ClintInterface, Bbox
from smartcar.paddlebaidu.ernie_bot import (
    ErnieBotWrap,
    ActionPrompt,
    HumAttrPrompt,
    ImagePrompt,
    OrderPrompt,
)
from smartcar.whalesbot.tools import CountRecord, get_yaml, IndexWrap
import sys
from typing import List
import re

from smartcar.whalesbot.vehicle.base.controller_wrap import PoutD

# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from smartcar import logger


def filter_chinese_letter(text: str) -> str:
    # 正则：汉字 \u4e00-\u9fff + 大小写字母 a-zA-Z
    res = re.findall(r"[\u4e00-\u9fffa-zA-Z]", text)
    return "".join(res)


def sellect_program(programs, order, win_order):
    """
    选择程序并生成显示字符串

    该函数用于生成程序选择菜单的显示字符串，突出显示当前选中的程序。

    参数:
        programs: 程序列表，包含所有可选择的程序
        order: 当前选中的程序索引
        win_order: 窗口起始索引

    返回:
        str: 生成的显示字符串，包含程序列表和当前选中的程序标记
    """
    dis_str = ""
    start_index = 0

    start_index = order - win_order
    for i, program in enumerate(programs):
        if i < start_index:
            continue

        now = str(program)
        if i == order:
            now = f">>{i + 1}.{now}"
        else:
            now = f"  {i + 1}.{now}"
        if len(now) >= 19:
            now = now[:19]
        else:
            now = now + "\n"
        dis_str += now
        if i - start_index == 4:
            break
    return dis_str


def kill_other_python():
    """
    终止其他Python进程

    该函数用于终止除当前进程外的其他Python进程，以避免进程冲突。

    注意:
        该函数会强制终止其他Python进程，请谨慎使用。
    """

    pid_me = os.getpid()
    # logger.info("my pid ", pid_me, type(pid_me))
    python_processes = []
    for proc in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            if (
                "python" in proc.info["name"].lower()
                and len(proc.info["cmdline"]) > 1
                and len(proc.info["cmdline"][1]) < 30
            ):
                python_processes.append(proc.info)
        # 出现异常的时候捕获 不存在的异常，权限不足的异常， 僵尸进程
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    for process in python_processes:
        # logger.info(f"PID: {process['pid']}, Name: {process['name']}, Cmdline: {process['cmdline']}")
        # logger.info("this", process['pid'], type(process['pid']))
        if int(process["pid"]) != pid_me:
            os.kill(int(process["pid"]), signal.SIGKILL)
            time.sleep(0.3)


def limit(value, value_range):
    """
    限制值在指定范围内

    该函数用于将输入值限制在[-value_range, value_range]范围内。

    参数:
        value: 输入值
        value_range: 范围上限

    返回:
        float: 限制后的值
    """
    return max(min(value, value_range), 0 - value_range)


# 两个pid集合成一个
class PidCal2:
    """
    PID控制器集合类

    该类包含两个PID控制器，分别用于y轴和角度控制。
    """

    def __init__(self, cfg_pid_y, cfg_pid_angle):
        """
        初始化PID控制器集合

        参数:
            cfg_pid_y: y轴PID控制器的配置参数
            cfg_pid_angle: 角度PID控制器的配置参数
        """
        self.pid_y = PID(**cfg_pid_y)
        self.pid_angle = PID(**cfg_pid_angle)

    def get_out(self, error_y, error_angle):
        """
        计算PID输出

        参数:
            error_y: y轴误差
            error_angle: 角度误差

        返回:
            tuple: (y轴PID输出, 角度PID输出)
        """
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out


class LanePidCal:
    """
    车道PID控制器类

    该类用于车道保持控制，包含y轴和角度PID控制器。
    """

    def __init__(self, cfg_pid_y, cfg_pid_angle):
        """
        初始化车道PID控制器

        参数:
            cfg_pid_y: y轴PID控制器的配置参数
            cfg_pid_angle: 角度PID控制器的配置参数
        """
        # y_out_limit = 0.7
        # self.pid_y = PID(5, 0, 0)
        # self.pid_y.setpoint = 0
        # self.pid_y.output_limits = (-y_out_limit, y_out_limit)
        # print(cfg_pid_y)
        # print(cfg_pid_angle)
        self.pid_y = PID(**cfg_pid_y)
        # print(self.pid_y)

        angle_out_limit = 1.5
        self.pid_angle = PID(3, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)

    def get_out(self, error_y, error_angle):
        """
        计算PID输出

        参数:
            error_y: y轴误差
            error_angle: 角度误差

        返回:
            tuple: (y轴PID输出, 角度PID输出)
        """
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out


class DetPidCal:
    """
    检测PID控制器类

    该类用于目标检测控制，包含y轴和角度PID控制器。
    """

    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        """
        初始化检测PID控制器

        参数:
            cfg_pid_y: y轴PID控制器的配置参数（可选）
            cfg_pid_angle: 角度PID控制器的配置参数（可选）
        """
        y_out_limit = 0.7
        self.pid_y = PID(0.3, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        angle_out_limit = 1.5
        self.pid_angle = PID(2, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)

    def get_out(self, error_y, error_angle):
        """
        计算PID输出

        参数:
            error_y: y轴误差
            error_angle: 角度误差

        返回:
            tuple: (y轴PID输出, 角度PID输出)
        """
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out


class LocatePidCal:
    """
    定位PID控制器类

    该类用于位置定位控制，包含x轴和y轴PID控制器。
    """

    def __init__(self):
        """
        初始化定位PID控制器

        初始化x轴和y轴的PID控制器，设置默认参数和输出限制。
        """
        y_out_limit = 0.3
        self.pid_y = PID(0.5, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        x_out_limit = 0.3
        self.pid_x = PID(0.5, 0, 0)
        self.pid_x.setpoint = 0
        self.pid_x.output_limits = (-x_out_limit, x_out_limit)

    def set_target(self, x, y):
        """
        设置目标位置

        参数:
            x: x轴目标位置
            y: y轴目标位置
        """
        self.pid_y.setpoint = y
        self.pid_x.setpoint = x

    def get_out(self, error_x, error_y):
        """
        计算PID输出

        参数:
            error_x: x轴误差
            error_y: y轴误差

        返回:
            tuple: (x轴PID输出, y轴PID输出)
        """
        pid_y_out = self.pid_y(error_y)
        pid_x_out = self.pid_x(error_x)
        return pid_x_out, pid_y_out


class MyCar(MecanumDriver):
    """
    智能车控制类

    该类继承自MecanumDriver，实现了智能车的完整控制功能，包括传感器初始化、PID控制、摄像头控制、
    目标检测、车道保持等功能。
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
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
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

    def paddle_infer_init(self):
        """
        初始化Paddle推理

        初始化车道保持、前置方向识别、任务识别和OCR识别的推理接口。
        """
        # 前置巡线
        self.crusie = ClintInterface("lane")
        # 前置左右方向识别
        # self.front_det = ClintInterface('front')
        # 任务识别
        self.task_det = ClintInterface("task")
        # ocr识别
        self.ocr_rec = ClintInterface("ocr")
        # 识别为None
        self.last_det = None

    def ernie_bot_init(self):
        """
        初始化文心一言分析

        初始化、图像分析和订单分析的文心一言接口。
        """
        self.image_analysis = ErnieBotWrap()

        self.order_analysis = ErnieBotWrap()
        self.order_analysis.set_promt(str(OrderPrompt()))

    def animal_image_analysis(self):
        dets = self.get_detection_results()
        if len(dets) <= 0:
            print("未检测到任何目标，无法裁剪")
            return None, None
        cls_id, det_id, label, score, x_c, y_c, w, h = dets[0]
        image = self.side_image.copy()

        # 将归一化坐标转换为像素坐标
        img_h, img_w = image.shape[:2]
        x_c = int((x_c + 1) / 2 * img_w)
        y_c = int((y_c + 1) / 2 * img_h)
        w = int(w * img_w / 2)
        h = int(h * img_h / 2)
        x1 = int(x_c - w / 2)
        y1 = int(y_c - h / 2)
        x2 = int(x_c + w / 2)
        y2 = int(y_c + h / 2)

        # img_h, img_w = image.shape[:2]

        # # 计算坐标 + 强制边界保护（核心修复！）
        # x1 = int(max(0, x_c - w / 2))
        # y1 = int(max(0, y_c - h / 2))
        # x2 = int(min(img_w, x_c + w / 2))
        # y2 = int(min(img_h, y_c + h / 2))
        # 防止裁剪出空图（核心修复！）
        if x2 <= x1 or y2 <= y1:
            print("裁剪区域无效，跳过")
            return None, None
        cropped_img = image[y1:y2, x1:x2]

        _, img_encoded = cv2.imencode(".jpg", cropped_img)
        # 转 base64 字符串
        base64_image = base64.b64encode(img_encoded.tobytes()).decode("utf-8")

        result, analysis = self.image_analysis.get_image_res(base64_image)
        print(f"image result: {result}  \nanalysis:{analysis}")
        return result, analysis

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

    # 延时函数
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

    # 按键检测线程
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

    # 根据某个值获取列表中匹配的结果
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

    def move_base(self, sp, end_fuction, stop=STOP_PARAM):
        """
        基础移动方法

        设置车辆速度并持续移动，直到满足结束条件。

        参数:
            sp: 速度向量 [x, y, z]
            end_fuction: 结束条件函数，返回True时停止移动
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        self.set_velocity(sp[0], sp[1], sp[2])
        while True:
            if self._stop_flag:
                return
            if end_fuction():
                break
            self.set_velocity(sp[0], sp[1], sp[2])
        if stop:
            self.set_velocity(0, 0, 0)

    #  高级移动，按着给定速度进行移动，直到满足条件
    # def move_advance(self, sp, value_h=None, value_l=None, times=1, sides=1, dis_out=0.2, stop=STOP_PARAM):
    #     """
    #     高级移动方法

    #     按照给定速度移动，直到满足传感器条件。

    #     参数:
    #         sp: 速度向量 [x, y, z]
    #         value_h: 传感器上限值，默认为1200
    #         value_l: 传感器下限值，默认为0
    #         times: 重复次数，默认为1
    #         sides: 传感器选择，1为左侧，-1为右侧
    #         dis_out: 距离限制，默认为0.2
    #         stop: 是否在结束后停止车辆，默认为STOP_PARAM
    #     """
    #     if value_h is None:
    #         value_h = 1200
    #     if value_l is None:
    #         value_l = 0
    #     # _sensor_usr = self.left_sensor
    #     # if sides == -1:
    #     #     _sensor_usr = self.right_sensor
    #     # 用于检测开始过渡部分的标记
    #     flag_start = False
    #     def end_fuction():
    #         nonlocal flag_start
    #         val_sensor = _sensor_usr.read()
    #         # print("val:", val_sensor)
    #         if val_sensor < value_h and val_sensor > value_l:
    #             return flag_start
    #         else:
    #             flag_start = True
    #             return False
    #     for i in range(times):
    #         self.move_base(sp, end_fuction, stop=False)
    #     if stop:
    #         self.stop()

    def move_time(self, sp, dur_time=1, stop=STOP_PARAM):
        """
        按时间移动

        以给定速度移动指定的时间。

        参数:
            sp: 速度向量 [x, y, z]
            dur_time: 移动时间（秒），默认为1
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        self.set_velocity_for_duration(sp[0], sp[1], sp[2], dur_time)
        if stop:
            self.stop()

    def move_distance(self, sp, dis=0.1, stop=STOP_PARAM):
        """
        按距离移动

        以给定速度移动指定的距离。

        参数:
            sp: 速度向量 [x, y, z]
            dis: 移动距离，默认为0.1
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        end_dis = self.get_distance() + dis

        def end_func():
            return self.get_distance() > end_dis

        self.move_base(sp, end_func, stop)

    # 计算两个坐标的距离
    def calculation_dis(self, pos_dst, pos_src):
        """
        计算两个坐标的距离

        计算两个二维坐标点之间的欧几里得距离。

        参数:
            pos_dst: 目标坐标 [x, y]
            pos_src: 源坐标 [x, y]

        返回:
            float: 两个坐标之间的距离
        """
        return math.sqrt(
            (pos_dst[0] - pos_src[0]) ** 2 + (pos_dst[1] - pos_src[1]) ** 2
        )

    def det2pose(self, det, w_r=0.06):
        """
        将检测结果转换为真实世界坐标

        根据检测结果和物体实际宽度，计算物体在真实世界中的坐标和距离。

        参数:
            det: 检测结果，包含 [x, y, w, h]（归一化坐标）
            w_r: 物体实际宽度（米），默认为0.06

        返回:
            tuple: (x坐标, y坐标, 距离)，单位为米
        """
        # r 真实  v 成像  f 焦点
        # rf 真实到焦点的距离  vf 相到焦点的距离
        vf_dis = 1.445
        x_v, y_v, w_v, h_v = det

        rf_dis = vf_dis * w_r / w_v
        x_r = x_v * rf_dis / vf_dis
        y_r = y_v * rf_dis / vf_dis
        return x_r, y_r, rf_dis

    # 侧面摄像头进行位置定位
    def lane_det_location(
        self,
        speed,
        pts_tar=[[0, 70, "text_det", 0, 0, 0, 0.70, 0.70]],
        dis_out=0.05,
        side=1,
        time_out=2,
        det="task",
    ):
        """
        侧面摄像头进行位置定位

        使用侧面摄像头检测目标并进行位置定位，通过PID控制调整车辆位置。

        参数:
            speed: 移动速度
            pts_tar: 目标点列表，每个元素包含 [id, 宽度, 标签, 置信度, x, y, w, h]
            dis_out: 距离限制，默认为0.05
            side: 方向，1为正方向，-1为反方向
            time_out: 超时时间（秒），默认为2
            det: 检测类型，默认为'task'

        返回:
            int: 目标索引，如果超时或距离超出限制则返回False
        """
        end_time = time.time() + time_out
        infer = self.task_det
        loc_pid = get_yaml(self.yaml_path)["location_pid"]  # type: ignore
        pid_x = PID(**loc_pid["pid_x"])
        pid_x.output_limits = (-speed, speed)
        pid_y = PID(**loc_pid["pid_y"])
        pid_y.output_limits = (-0.15, 0.15)
        # pid_w = PID(1.0, 0, 0.00, setpoint=0, output_limits=(-0.15, 0.15))

        # 用于相同记录结果的计数类
        x_count = CountRecord(5)
        dis_count = CountRecord(5)

        out_x = speed
        out_y = 0

        # 此时设置相对初始位置
        # self.set_pos_relative()
        # self.dis_tra_st = self.get_distance()
        x_st, y_st, _ = self.get_odometry()
        find_tar = False
        tar = []
        for pt_tar in pts_tar:
            # id, 物体宽度，置信度, 归一化bbox[x_c, y_c, w, h]
            tar_id, tar_width, tar_label, tar_score, tar_bbox = (
                pt_tar[0],
                pt_tar[1],
                pt_tar[2],
                pt_tar[3],
                pt_tar[4:],
            )
            tar_width *= 0.001
            tar_x, tar_y, tar_dis = self.det2pose(tar_bbox, tar_width)
            tar.append([tar_id, tar_width, tar_x, tar_y, tar_dis])
        # logger.info("tar x:{} dis:{}".format(tar_x, tar_dis))
        tar_id, tar_width, tar_x, tar_y, tar_dis = tar[0]
        pid_x.setpoint = tar_x
        pid_y.setpoint = tar_dis
        tar_index = 0
        flag_location = False
        while True:
            if self._stop_flag:
                return
            if time.time() > end_time:
                logger.info("time out")
                self.set_velocity(0, 0, 0)
                return False
            _pos_x, _pos_y, _pos_omage = self.get_odometry()  # 用来计算距离

            if abs(_pos_x - x_st) > dis_out or abs(_pos_y - y_st) > dis_out:
                if not find_tar:
                    logger.info("task location dis out")
                    self.set_velocity(0, 0, 0)
                    return False
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)

            img_side_show = img_side.copy()
            for det in dets_ret:
                det_cls_id, det_id, det_label, det_score, det_bbox = (
                    det[0],
                    det[1],
                    det[2],
                    det[3],
                    det[4:],
                )
                x_c, y_c, w, h = det_bbox
                # 将归一化坐标转换为像素坐标
                img_h, img_w = img_side.shape[:2]
                x_c = int((x_c + 1) / 2 * img_w)
                y_c = int((y_c + 1) / 2 * img_h)
                w = int(w * img_w / 2)
                h = int(h * img_h / 2)
                x1 = int(x_c - w / 2)
                y1 = int(y_c - h / 2)
                x2 = int(x_c + w / 2)
                y2 = int(y_c + h / 2)
                # 绘制矩形框
                cv2.rectangle(img_side_show, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 绘制标签
                label_text = f"{det_label}:{det_score:.2f}"
                cv2.putText(
                    img_side_show,
                    label_text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            self.streamer.update_frame(img_side_show, "cam2")

            # dets_ret = self.mot_hum(img_side)
            # cv2.imshow("side", img_side)
            # cv2.waitKey(1)

            # 进行排序，此处排列按照自中心由近及远的顺序
            dets_ret.sort(key=lambda x: (x[4]) ** 2 + (x[5]) ** 2)
            print(dets_ret)
            # # 找到最近对应的类别，类别存在第一个位置
            # det = self.get_list_by_val(dets_ret, 2, tar_label)

            # 如果没有，就重新获取
            if len(dets_ret) > 0:
                det = dets_ret[0]
                # 结果分解
                det_id, obj_id, det_label, det_score, det_bbox = (
                    det[0],
                    det[1],
                    det[2],
                    det[3],
                    det[4:],
                )
                # if find_tar is False:
                # tar_index = 0
                # for tar_pt in tar:
                for index, tar_pt in enumerate(tar):
                    if det_id == tar_pt[0]:
                        tar_index = index
                        tar_id, tar_width, tar_x, tar_y, tar_dis = tar_pt
                        pid_x.setpoint = tar_x
                        pid_y.setpoint = tar_dis
                        find_tar = True
                        # print("find tar", tar_id)
                        break

                if det_id == tar_id:
                    _x, _y, _dis = self.det2pose(det_bbox, tar_width)
                    out_x = pid_x(_x) * side  # type: ignore
                    out_y = pid_y(_dis) * side  # pyright: ignore[reportOptionalOperand]
                    # out_y = pid_y(_dis)
                    # out_y = pid_w(bbox_error[2])
                    # 检测偏差值连续小于阈值时，跳出循环
                    # print(bbox_error)
                    # print("err x:{:.2}, dis:{:.2}, tar x:{:.2}, tar dis:{:.2}".format(_x, _dis, tar_x, tar_dis))
                    flag_x = x_count(abs(_x - tar_x) < 0.01)
                    flag_dis = dis_count(abs(_dis - tar_dis) < 0.01)
                    if flag_x:
                        out_x = 0
                    if flag_dis:
                        out_y = 0
                    if flag_x and flag_dis:
                        logger.info("location{} ok".format(tar_id))
                        # flag_location = True
                        # 停止
                        self.set_velocity(0, 0, 0)
                        return tar_index

                # print("error_x:{:.2}, error_y:{:.2}, out_x:{:.2}, out_y:{:2}".format(bbox_error[0], bbox_error[2], out_x, out_y))
            else:
                x_count(False)
                dis_count(False)
            self.set_velocity(out_x, out_y, 0)

    def lane_base(self, speed, end_fuction, stop=STOP_PARAM):
        """
        车道保持基础方法

        使用前置摄像头进行车道检测和保持，根据检测结果调整车辆方向。

        参数:
            speed: 行驶速度
            end_fuction: 结束条件函数，返回True时停止
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        while True:
            if self._stop_flag:
                return

            error_y, error_angle = self.get_lane_results()
            y_speed, angle_speed = self.lane_pid.get_out(-error_y, -error_angle)
            self.set_velocity(speed, y_speed, angle_speed)
            if end_fuction():
                break
        if stop:
            self.stop()

    # def lane_det_base(self, speed, end_fuction, stop=STOP_PARAM):
    #     """
    #     目标检测基础方法

    #     使用前置摄像头进行目标检测，根据检测结果调整车辆方向。

    #     参数:
    #         speed: 行驶速度
    #         end_fuction: 结束条件函数，接收距离参数，返回True时停止
    #         stop: 是否在结束后停止车辆，默认为STOP_PARAM
    #     """
    #     # 初始化速度和角度速度
    #     y_speed = 0
    #     angle_speed = 0
    #     w_r=0.06
    #     # 无限循环
    #     while True:
    #         # 读取前摄像头图像
    #         image = self.cap_front.read()
    #         self.streamer.update_frame(image,"cam1")
    #         dets_ret = self.front_det(image)
    #         # 此处检测简单不需要排序
    #         # dets_ret.sort(key=lambda x: x[4]**2 + (x[5])**2)
    #         if len(dets_ret)>0:
    #             det = dets_ret[0]
    #             det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
    #             _x, _y, _dis = self.det2pose(det_bbox, w_r)
    #             # error_y = det_bbox[0]
    #             # dis_x = 1 - det_bbox[1]
    #             if end_fuction(_dis):
    #                 break
    #             error_angle = _x /_dis
    #             y_speed, angle_speed = self.det_pid.get_out(_x, error_angle)
    #             # print("_x:{:.2}, _angle:{:.2}, y_vel:{:.2}, angle_vel:{:.2}, dis{:.2}".format(_x, error_angle, y_speed, angle_speed, _dis))
    #         self.set_velocity(speed, y_speed, angle_speed)
    #         # if end_fuction(0):
    #         #     break
    #     if stop:
    #         self.stop()

    # def lane_det_time(self, speed, time_dur, stop=STOP_PARAM):
    #     """
    #     目标检测定时方法

    #     使用前置摄像头进行目标检测，持续指定的时间。

    #     参数:
    #         speed: 行驶速度
    #         time_dur: 持续时间（秒）
    #         stop: 是否在结束后停止车辆，默认为STOP_PARAM
    #     """
    #     time_end = time.time() + time_dur
    #     end_fuction = lambda x: time.time() > time_end
    #     self.lane_det_base(speed, end_fuction, stop=stop)

    # def lane_det_dis2pt(self, speed, dis_end, stop=STOP_PARAM):
    #     """
    #     目标检测定距方法

    #     使用前置摄像头进行目标检测，直到与目标的距离小于指定值。

    #     参数:
    #         speed: 行驶速度
    #         dis_end: 目标距离阈值
    #         stop: 是否在结束后停止车辆，默认为STOP_PARAM
    #     """
    #     # lambda定义endfunction
    #     end_fuction = lambda x: x < dis_end and x != 0
    #     self.lane_det_base(speed, end_fuction, stop=stop)

    def lane_time(self, speed, time_dur, stop=STOP_PARAM):
        """
        车道保持定时方法

        使用前置摄像头进行车道保持，持续指定的时间。

        参数:
            speed: 行驶速度
            time_dur: 持续时间（秒）
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        time_end = time.time() + time_dur

        def end_fuction():
            return time.time() > time_end

        self.lane_base(speed, end_fuction, stop=stop)

    # 巡航一段路程
    def lane_dis(self, speed, dis_end, stop=STOP_PARAM):
        """
        车道保持定距方法

        使用前置摄像头进行车道保持，直到行驶距离超过指定值。

        参数:
            speed: 行驶速度
            dis_end: 目标距离
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """

        # lambda重新endfunction
        def end_fuction():
            return self.get_distance() > dis_end

        self.lane_base(speed, end_fuction, stop=stop)

    def lane_dis_offset(self, speed, dis_hold, stop=STOP_PARAM):
        """
        车道保持距离偏移方法

        使用前置摄像头进行车道保持，行驶指定的距离偏移量。

        参数:
            speed: 行驶速度
            dis_hold: 距离偏移量
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        dis_start = self.get_distance()
        dis_stop = dis_start + dis_hold
        self.lane_dis(speed, dis_stop, stop=stop)

    # def lane_sensor(self, speed, value_h=None, value_l=None, dis_offset=0.0, times=1, sides=1, stop=STOP_PARAM):
    #     """
    #     车道保持传感器方法

    #     使用前置摄像头进行车道保持，直到传感器检测到指定范围的值。

    #     参数:
    #         speed: 行驶速度
    #         value_h: 传感器上限值，默认为1200
    #         value_l: 传感器下限值，默认为0
    #         dis_offset: 距离偏移量，默认为0.0
    #         times: 重复次数，默认为1
    #         sides: 传感器选择，1为左侧，-1为右侧
    #         stop: 是否在结束后停止车辆，默认为STOP_PARAM
    #     """
    #     if value_h is None:
    #         value_h = 1200
    #     if value_l is None:
    #         value_l = 0
    #     # _sensor_usr = self.left_sensor
    #     # if sides == -1:
    #     #     _sensor_usr = self.right_sensor
    #     # 用于检测开始过渡部分的标记
    #     flag_start = False
    #     def end_fuction():
    #         nonlocal flag_start
    #         # val_sensor = _sensor_usr.read()
    #         # print("val:", val_sensor)
    #         if val_sensor < value_h and val_sensor > value_l:
    #             return flag_start
    #         else:
    #             flag_start = True
    #             return False

    #     for i in range(times):
    #         self.lane_base(speed, end_fuction, stop=False)
    #     # 根据需要是否巡航
    #     self.lane_dis_offset(speed, dis_offset, stop=stop)

    # def get_card_side(self):
    #     """
    #     检测卡片左右指示

    #     使用前置摄像头检测卡片上的左右指示，返回相应的方向。

    #     返回:
    #         int: -1表示右转，1表示左转，0表示停止或未检测到
    #     """
    #     # 检测卡片左右指示
    #     count_side = CountRecord(3)
    #     while True:
    #         if self._stop_flag:
    #             return 0
    #         image = self.cap_front.read()
    #         dets_ret = self.front_det(image)
    #         if len(dets_ret) == 0:
    #             count_side(-1)
    #             continue
    #         det = dets_ret[0]
    #         det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
    #         # 联系检测超过3次
    #         if count_side(det_label):
    #             if det_label == 'turn_right':
    #                 return -1
    #             elif det_label == 'turn_left':
    #                 return 1
    def get_det_ocr(self, det, label="name", time_out=5.0):
        time_stop = time.time() + time_out
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        text_count = CountRecord(3)
        text_out = None
        print(det)
        while True:
            if self._stop_flag:
                return text_out
            if time.time() > time_stop:
                return text_out
            img = self.side_image
            if det is not None:
                det_cls_id, det_id, det_label, det_score, det_bbox = (
                    det[0],
                    det[1],
                    det[2],
                    det[3],
                    det[4:],
                )
                if label is not None:
                    flag = det_label == label
                else:
                    flag = det_label == "order" or det_label == "name"
                if flag:
                    # x1, y1, w, h = det_bbox
                    # # print(img.shape)
                    # # print(x1, y1, w, h)
                    # x1 = img.shape[1] * (1+x1) / 2 - img.shape[1] * w / 4
                    # x2 = x1 + img.shape[1] * w / 2
                    # y1 = img.shape[0] * (1+y1) / 2 - img.shape[0] * h / 4
                    # y2 = y1 + img.shape[0] * h / 2
                    # x1 = 0 if x1 < 0 else int(x1)
                    # x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                    # y1 = 0 if y1 < 0 else int(y1)
                    # y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                    # # print(x1, x2, y1, y2)

                    # 将归一化坐标转换为像素坐标
                    x_c, y_c, w, h = det_bbox
                    w *= 1.2
                    h *= 1.2
                    img_h, img_w = img.shape[:2]
                    x_c = int((x_c + 1) / 2 * img_w)
                    y_c = int((y_c + 1) / 2 * img_h)
                    w = int(w * img_w / 2)
                    h = int(h * img_h / 2)
                    x1 = int(x_c - w / 2)
                    y1 = int(y_c - h / 2)
                    x2 = int(x_c + w / 2)
                    y2 = int(y_c + h / 2)

                    img_txt = img[y1:y2, x1:x2]

                    self.streamer.update_frame(img_txt, "cam1")
                    text = self.ocr_rec(img_txt)
                    print(f"当前检测文本: {text}")
                    text = "".join(re.findall(r"[\u4e00-\u9fffa-zA-Z]", text))
                    print(f"整理后文本: {text}")
                    if text_out is None:
                        text_out = text
                    else:
                        # 文本相似度比较
                        matcher = difflib.SequenceMatcher(None, text_out, text).ratio()
                        if text_count(matcher > 0.85):
                            return text_out
                        else:
                            text_out = text

    def get_ocr(self, label=None, time_out=3.0):
        """
        进行OCR识别

        使用侧面摄像头获取图像，进行文本检测和OCR识别，返回识别结果。

        参数:
            time_out: 超时时间（秒），默认为3

        返回:
            str: 识别到的文本，如果超时或未检测到则返回None
        """
        time_stop = time.time() + time_out
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        text_count = CountRecord(3)
        text_out = None
        while True:
            if self._stop_flag:
                return
            if time.time() > time_stop:
                return None
            dets = self.get_detection_results()

            img = self.side_image
            if len(dets) > 0:
                for det in dets:
                    det_cls_id, det_id, det_label, det_score, det_bbox = (
                        det[0],
                        det[1],
                        det[2],
                        det[3],
                        det[4:],
                    )
                    if label is not None:
                        flag = det_label == label
                    else:
                        flag = det_label == "order" or det_label == "name"
                    if flag:
                        # x1, y1, w, h = det_bbox

                        # # print(img.shape)
                        # # print(x1, y1, w, h)
                        # x1 = img.shape[1] * (1 + x1) / 2 - img.shape[1] * w / 4
                        # x2 = x1 + img.shape[1] * w / 2
                        # y1 = img.shape[0] * (1 + y1) / 2 - img.shape[0] * w / 4
                        # y2 = y1 + img.shape[0] * h / 2
                        # x1 = 0 if x1 < 0 else int(x1)
                        # x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                        # y1 = 0 if y1 < 0 else int(y1)
                        # y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                        # # print(x1, x2, y1, y2)
                        # img_txt = img[y1:y2, x1:x2]
                                            # 将归一化坐标转换为像素坐标
                        x_c, y_c, w, h = det_bbox
                        w *= 1.1
                        h *= 1.1
                        img_h, img_w = img.shape[:2]
                        x_c = int((x_c + 1) / 2 * img_w)
                        y_c = int((y_c + 1) / 2 * img_h)
                        w = int(w * img_w / 2)
                        h = int(h * img_h / 2)
                        x1 = int(x_c - w / 2)
                        y1 = int(y_c - h / 2)
                        x2 = int(x_c + w / 2)
                        y2 = int(y_c + h / 2)

                        img_txt = img[y1:y2, x1:x2]
                        self.streamer.update_frame(img_txt, "cam1")

                        text = self.ocr_rec(img_txt)
                        if text_out is None:
                            text_out = text
                        else:
                            # 文本相似度比较
                            matcher = difflib.SequenceMatcher(
                                None, text_out, text
                            ).ratio()
                            if text_count(matcher > 0.85):
                                return text_out
                            else:
                                text_out = text
                            # if matcher > 0.85:
                            #     text_count(T)
                        # print(text)
                        # print(res.bbox)
                        # print(text)
                        # if text_count(text):
                        #     return text

    def yiyan_get_humattr(self, text):
        """
        获取人类属性分析

        使用文心一言分析文本中的人类属性信息。

        参数:
            text: 包含人类属性信息的文本

        返回:
            dict: 人类属性分析结果
        """
        return self.hum_analysis.get_res_json(text)

    def yiyan_get_actions(self, text):
        """
        获取动作分析

        使用文心一言分析文本中的动作信息。

        参数:
            text: 包含动作信息的文本

        返回:
            dict: 动作分析结果
        """
        return self.action_bot.get_res_json(text)

    def draw_detection_results(self, img, dets_ret):
        """
        将检测结果绘制在图像上

        Args:
            img: 原始图像
            dets_ret: 检测结果列表，每个元素包含 [cls_id, det_id, label, score, x_c, y_c, w, h]

        Returns:
            绘制了检测结果的图像
        """
        # 创建图像副本，避免修改原始图像
        img_show = img.copy()

        # 遍历每个检测结果
        for index, det in enumerate(dets_ret):
            # [cls_id:6 obj_id:0 label:water_l2 score:0.955 bbox:[309 334 399 431]]
            det_cls_id, det_id, det_label, det_score, det_bbox = (
                det[0],
                det[1],
                det[2],
                det[3],
                det[4:],
            )
            x_c, y_c, w, h = det_bbox

            # 将归一化坐标转换为像素坐标
            img_h, img_w = img.shape[:2]
            x_c = int((x_c + 1) / 2 * img_w)
            y_c = int((y_c + 1) / 2 * img_h)
            w = int(w * img_w / 2)
            h = int(h * img_h / 2)
            x1 = int(x_c - w / 2)
            y1 = int(y_c - h / 2)
            x2 = int(x_c + w / 2)
            y2 = int(y_c + h / 2)

            # 绘制矩形框
            cv2.rectangle(img_show, (x1, y1), (x2, y2), (0, 255, 0), 1)

            # 绘制标签
            label_text = f"{index}-{det_label}:{det_score:.2f}"
            cv2.putText(
                img_show,
                label_text,
                (x1, y1),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        return img_show

    def get_detection_results(
        self, sort_pos=(0, 0), limit_x=1, limit_y=1
    ) -> List[list]:
        """
        获取检测结果,使用任务的目标检测对侧边摄像头图像进行检测，返回检测结果。

        返回:
            list: - 检测结果列表，每个元素包含 [cls_id, det_id, label, score, x_c, y_c, w, h]
        """
        self.side_image = self.cap_side.read()
        image = self.side_image.copy()
        det_task = self.task_det(image)
        det_task = [det for det in det_task if abs(det[4]) <= limit_x]
        det_task = [det for det in det_task if abs(det[5]) <= limit_y]

        det_task.sort(
            key=lambda x: (x[4] - sort_pos[0]) ** 2 + (x[5] - sort_pos[1]) ** 2
        )  # 按照距离由近及远排序
        image = self.draw_detection_results(image, det_task)
        self.streamer.update_frame(image, "cam2")
        # print(det_task)
        return det_task

    def get_lane_results(self):
        image = self.cap_front.read().copy()
        res = self.crusie(image)
        error, angle = res[0], res[1]
        # 绘制标签
        label_text = f"d_e: {error:7.5f} d_a:{angle:7.5f}"

        cv2.putText(
            image,
            label_text,
            (20, 40),
            cv2.FONT_HERSHEY_TRIPLEX,
            1.0,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            label_text,
            (20, 40),
            cv2.FONT_HERSHEY_TRIPLEX,
            1.0,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        self.streamer.update_frame(image, "cam1")
        # print(label_text)
        return error, angle

    def get_target_location(self, det):
        """
        通过传入的目标在图像的坐标，计算目标相对小车的偏移 x,y

        参数:
            det: 包含目标检测信息的列表，格式为 [cls_id, obj_id,label, score, x_c, y_c, w, h]
                - x_c: 目标在图像中的 x 坐标
                - y_c: 目标在图像中的 y 坐标
                - w: 目标的宽度
                - h: 目标的高度

        返回:
            tuple: 目标相对小车的坐标 (loc_x, loc_y)
                - loc_x: 目标相对小车的 x 坐标
                - loc_y: 目标相对小车的 y 坐标
        """
        # 摄像头图像在现实中实际的高和宽
        CAMERA_HEIGHT = 0.23
        CAMERA_WIDTH = 0.33
        # 机械臂x原点距离小车中心的距离
        ARM_OFFSET = 0.15

        # 获取机械臂的方向和长度
        arm_y = self.arm.x_pose_now + ARM_OFFSET
        side = self.arm.side
        length = 0

        # 根据机械臂方向调整长度
        if side == "RIGHT":
            length = -self.arm.arm_length
        elif side == "LEFT":
            length = self.arm.arm_length

        # 提取目标在图像中的坐标和尺寸
        x_c, y_c, w, h = det[4:]

        # 计算目标中心点在摄像头中的世界坐标
        x = CAMERA_WIDTH * (x_c + w / 2)
        y = CAMERA_HEIGHT * (y_c + h / 2)

        # 计算目标中心点在小车中的世界坐标
        loc_x = x
        loc_y = y + arm_y + length

        return loc_x, loc_y

    def move_to_detection_target(
        self,
        delta_x=0.0,
        delta_y: Union[float, None] = 0.0,
        label=None,
        time_out=2.0,
        sort_pos=(0, 0),
        num=0,
    ):
        """
        前往目标位置

        参数:
            cls_id : 指定检测目标的 cls_id，默认None为距离中心最近的目标
            time_out: 设置超时时间
            包含目标检测信息的列表，格式为 [cls_id, obj_id,label, score, x_c, y_c, w, h]
        """
        time_stop = time.time() + time_out
        x_count = CountRecord(3)
        y_count = CountRecord(3)

        # pid_x.output_limits((-0.7, 0.7))

        out_x = 0
        out_y = 0
        # print(f"手柄方向：{self.arm.side}")
        if self.arm.side == "RIGHT":
            kp_y = -0.2
            kp_x = -0.25
            ki_x = -0.05
        else:
            kp_y = 0.2
            kp_x = 0.25
            ki_x = 0.05

        pid_x = PID(kp_x, ki_x)
        pid_x.setpoint = delta_x
        while True:
            if self._stop_flag:
                self.set_velocity(0, 0, 0)
                self.arm.x_speed(0)
                return -1, "None"

            dets = self.get_detection_results(sort_pos=sort_pos)

            if label is not None:
                dets = [item for item in dets if item[2] == label]

            if len(dets) > num:
                det = dets[num]
                dx, dy = det[4:6]
                # print(f"dx:{dx} dy:{dy}")
                out_x = -pid_x(dx)  # type: ignore
                if delta_y is None:
                    out_y = 0
                else:
                    out_y = kp_y * (dy - delta_y)

                flag_x = x_count(abs(dx) < 0.04)
                flag_y = y_count(abs(dy) < 0.02)
                if delta_y is None:
                    flag_y = True

                if flag_x:
                    out_x = 0
                if flag_y:
                    out_y = 0
                if flag_x and flag_y:
                    # logger.info(f"location{self.get_odometry()} ok, arm_pose{self.arm.x_pose_now}")
                    self.set_velocity(0, 0, 0)
                    self.arm.x_speed(0)
                    # return det[0],det[2]
            else:
                x_count(False)
                y_count(False)
            self.set_velocity(out_x, 0, 0)
            self.arm.x_speed(out_y)
            time.sleep(0.05)

            if time.time() > time_stop:
                self.set_velocity(0, 0, 0)
                self.arm.x_speed(0)
                logger.error("对齐目标超时")
                # logger.info(f"location{self.get_odometry()} ok, arm_pose{self.arm.x_pose_now}")

                try:
                    return det[0], det[2]
                except:
                    return (None, None)

    def adjust_arm_position(self, dis=0.05):
        # print(f"arm side:{self.arm.side}")
        x_position = self.arm.x_get_position()
        if self.arm.side == "LEFT":
            self.arm.move_x_position(x_position + dis)
        elif self.arm.side == "RIGHT":
            self.arm.move_x_position(x_position - dis)

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


def test_for_animal():
    try:
        while True:
            res = my_car.animal_image_analysis()
            print("\n\n")
            time.sleep(10)
    except KeyboardInterrupt:
        my_car.close()


if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    # arm = ArmController()
    time.sleep(1)

    my_car.arm.reset_position()
    # my_car.arm.set_arm_pose(0,0.2,"LEFT","DOWN")
    # my_car.debug(False)

    def ocr_test():
        print(my_car.get_ocr())

    my_car.manage([ocr_test])

    # my_car.lane_time(0.3, 5)

    # my_car.lane_dis_offset(0.3, 1.2)
    # my_car.lane_sensor(0.3, 0.5)
    # my_car.debug()

    # text = "犯人没有带着眼镜，穿着短袖"
    # criminal_attr = my_car.hum_analysis.get_res_json(text)
    # print(criminal_attr)
    # my_car.task.reset()
    # pt_tar = my_car.task.punish_crimall(arm_set=True)
    # hum_attr = my_car.get_hum_attr(pt_tar)
    # print(hum_attr)
    # res_bool = my_car.compare_humattr(criminal_attr, hum_attr)
    # print(res_bool)
    # pt_tar = [0, 1, 'pedestrian',  0, 0.02, 0.4, 0.22, 0.82]
    # for i in range(4):
    #     my_car.move_for([0.07, 0, 0])
    #     my_car.lane_det_location(0.1, pt_tar, det="mot", side=-1)
    # my_car.close()
    # text = my_car.get_ocr()
    # print(text)
    # pt_tar = my_car.task.pick_up_ball(arm_set=True)
    # my_car.lane_det_location(0.1, pt_tar)

    # my_car.debug()
    # while True:
    #     text = my_car.get_ocr()
    #     print(text)

    # my_car.task.reset()
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.lane_time(0.3, 5)
    # my_car.debug()

    # my_car.debug()

    # my_car.task.pick_up_block()
    # my_car.task.put_down_self_block()
    # my_car.lane_time(0.2, 2)
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.task.pick_up_block()
    # my_car.close()
    # logger.info(time.time())
    # my_car.lane_task_location(0.3, 2)

    # my_car.debug()
    # programs = [func1, func2, func3, func4, func5, func6]
    # my_car.manage(programs)
    # import sys
    # test_ord = 0
    # if len(sys.argv) >= 2:
    #     test_ord = int(sys.argv[1])
    # logger.info("test:", test_ord)
    # car_test(test_ord)
