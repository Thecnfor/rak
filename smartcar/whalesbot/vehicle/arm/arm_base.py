#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
机械臂控制模块

该模块实现了机械臂的运动控制, 包括竖直方向、水平方向的移动, 以及手部的控制。
XY 双轴运动能力已拆分到 arm_motion.ArmMotion mixin, 本文件保留 ArmController
的手部/姿态/复位/属性接口, 并组合 ArmMotion 以获得完整的机械臂能力。
"""

import math
import time
import numpy as np
import yaml
import os
import sys
from threading import Thread
from typing import Union

# 添加上本地目录
dir_this = os.path.abspath(os.path.dirname(__file__))
sys.path.append(dir_this)
# 添加上两层目录
dir_root = os.path.abspath(os.path.join(dir_this, "..", ".."))
sys.path.append(dir_root)

# 导入自定义模块
from ...tools import get_yaml, limit_val, CountRecord, PID, logger
from .. import AnalogInput, MotorWrap, Key4Btn, ServoPwm, ServoBus, StepperWrap, PoutD

# XY 轴运动能力(mixin)
from .arm_motion import (
    ArmMotion,
    POSITION_ERROR_THRESHOLD,
    STOP_CHECK_THRESHOLD,
    RESET_TIMEOUT,
)

# 常量定义(转发自 arm_motion, 保持外部引用兼容)
POSITION_ERROR_THRESHOLD = POSITION_ERROR_THRESHOLD  # 位置误差阈值
STOP_CHECK_THRESHOLD = STOP_CHECK_THRESHOLD  # 停止检查阈值
RESET_TIMEOUT = RESET_TIMEOUT  # 复位超时(秒)


def get_path_relative(*args):
    """
    获取相对路径

    Args:
        *args: 路径组件

    Returns:
        str: 完整的绝对路径
    """
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)


class ArmController(ArmMotion):
    """
    机械臂控制器, 组合 ArmMotion(XY 双轴运动) 与手部/姿态控制能力。
    """

    def __init__(self) -> None:
        """
        初始化机械臂控制类
        """
        self.yaml_path = get_path_relative("arm_cfg.yaml")

        with open(self.yaml_path, "r") as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        """机械臂的长度"""
        self.arm_length: float = self.config["arm_length"]
        # 初始化各部分参数(先 XY 轴以建立 motor_y/motor_x, 再手部, 最后位姿)
        self.y_params_init(**self.config["vert_cfg"])
        self.x_params_init(**self.config["horiz_cfg"])
        self.hand_params_init(**self.config["hand_cfg"])
        self.position_params_init(**self.config["pos_cfg"])

    def hand_params_init(self, hand, hand2, grap):
        """
        初始化手部参数

        Args:
            hand: 手臂舵机配置
            hand2: 手部舵机配置
            grap: 抓取机构配置
        """
        self.hand_servo = ServoPwm(hand2["port"], mode=hand2["mode"])
        self.hand_angle_list2 = hand2["angle_list"]
        self.arm_servo = ServoBus(hand["port"])
        self.hand_angle_list = hand["angle_list"]
        self.pump = PoutD(grap["port_pump"])
        self.valve = PoutD(grap["port_valve"])

    def grasp(self, value: bool):
        """
        控制抓取机构

        Args:
            value: 抓取状态, True为抓取, False为释放
        """
        self.pump.set(not value)
        self.valve.set(value)

    def position_params_init(self, pose_enable, pose_horiz, pose_vert, side):
        """
        初始化位置参数

        Args:
            pose_enable: 是否启用位置
            pose_horiz: 水平位置
            pose_vert: 竖直位置
            side: 方向
        """
        self.pose_enable = pose_enable
        self.y_pose_start = self.motor_y.get_dis() - pose_vert
        self.y_pose_now = pose_vert
        self.x_pose_start = self.motor_x.get_dis() - pose_horiz
        self.x_pose_now = pose_horiz
        self.side = side

    def save_config(self, pose_enable=True):
        """
        保存配置到YAML文件

        Args:
            pose_enable: 是否启用位置
        """
        self.config["pos_cfg"] = {
            "pose_enable": pose_enable,
            "pose_horiz": self.x_pose_now,
            "pose_vert": self.y_pose_now,
            "side": self.side,
        }
        with open(self.yaml_path, "w") as stream:
            yaml.dump(self.config, stream, sort_keys=False)

    def set_manually(self):
        """
        使用【4键】控制机械臂
        """
        self.key = Key4Btn(4)
        logger.info("Using 4 keys to control arm...")
        while True:
            value = self.key.get_key()
            if value == 1:
                self.y_speed(0.1)  # 向上
            elif value == 3:
                self.y_speed(-0.1)  # 向下
            elif value == 4:
                self.x_speed(0.1)  # 向右
            elif value == 2:
                self.x_speed(-0.1)  # 向左
            else:
                self.x_speed(0)
                self.y_speed(0)

    def reset_position(self):
        """
        重置机械臂位置
        """
        thread_reset_y = Thread(target=self.reset_y)
        thread_reset_x = Thread(target=self.reset_x)

        self.set_hand_angle("UP")
        self.set_arm_angle("RIGHT")
        thread_reset_y.daemon = True
        thread_reset_x.daemon = True
        thread_reset_y.start()
        thread_reset_x.start()
        thread_reset_y.join()
        thread_reset_x.join()
        # 回零兜底: 串口/传感器抖动时不回零也不应中断整个开机流程
        try:
            self.x = 0
            self.y = 0
        except Exception as e:
            logger.warning(f"机械臂回零失败({e}), 跳过(位姿可能不准)")
        self.save_config()

    def switch_side(self, side):
        """
        切换机械臂方向

        Args:
            side: 机械臂的方向, LEFT、RIGHT或MID
        """
        if self.side != side:
            self.side = side
            logger.info(f"Changing side to {self.side}")
        else:
            return
        angle_target = self.hand_angle_list[side]
        self.set_arm_angle(angle_target, 80)

    def set_arm_angle(self, angle: Union[str, int] = "RIGHT", speed=80):
        """
        设置机械臂角度

        Args:
            angle: 目标角度，可以是字符串（"LEFT", "MID", "RIGHT"）或数字
            speed: 速度
        """
        _angle = angle
        if isinstance(_angle, str):
            self.side = _angle
            assert _angle in (
                "LEFT",
                "MID",
                "RIGHT",
            ), "Direction should be LEFT, MID, or RIGHT"
            _angle = self.hand_angle_list[_angle]
        self._arm_angle_last = _angle
        self.arm_servo.set_angle(_angle, speed)

    def set_arm_angle_async(self, angle: Union[str, int] = "RIGHT", speed=80, callback=None):
        """
        异步设置机械臂角度(发命令不等应答), 供四轴并发使用。

        与 set_arm_angle 语义一致(同样支持 LEFT/MID/RIGHT 或数字), 仅发送方式不同,
        立即返回, 不等待舵机到达。
        """
        _angle = angle
        if isinstance(_angle, str):
            self.side = _angle
            assert _angle in (
                "LEFT",
                "MID",
                "RIGHT",
            ), "Direction should be LEFT, MID, or RIGHT"
            _angle = self.hand_angle_list[_angle]
        self._arm_angle_last = _angle
        self.arm_servo.set_angle_async(_angle, speed, callback=callback)

    def set_hand_angle(self, angle: Union[str, int] = "UP", speed=80):
        """
        设置机械臂手角度

        Args:
            angle: 目标角度，可以是字符串（"UP", "MID", "DOWN"）或数字
            speed: 速度
        """
        if isinstance(angle, str):
            assert angle in (
                "UP",
                "MID",
                "DOWN",
            ), "Direction should be UP, MID, or DOWN"
            angle = self.hand_angle_list2[angle]
        self._hand_angle_last = angle
        self.hand_servo.set_angle(angle, speed)

    def set_hand_angle_async(self, angle: Union[str, int] = "UP", speed=80, callback=None):
        """
        异步设置机械臂手角度(发命令不等应答), 供四轴并发使用。

        与 set_hand_angle 语义一致(同样支持 UP/MID/DOWN 或数字), 仅发送方式不同,
        立即返回, 不等待舵机到达。
        """
        if isinstance(angle, str):
            assert angle in (
                "UP",
                "MID",
                "DOWN",
            ), "Direction should be UP, MID, or DOWN"
            angle = self.hand_angle_list2[angle]
        self._hand_angle_last = angle
        self.hand_servo.set_angle_async(angle, speed, callback=callback)

    def set_arm_pose_async(self, x=None, y=None, arm=None, hand=None):
        """
        异步设置机械臂位姿(四轴并发)。

        与 set_arm_pose 语义一致: X/Y 双轴走 goto_position_async, arm/hand 舵机
        角度命令异步发出, 四个自由度并行开始。方法本身不阻塞主流程。

        用法:
            car.arm.set_arm_pose_async(x, y, arm="RIGHT", hand="DOWN")
            while not car.arm.goto_position_async(x, y):   # 等待 XY 到位
                car.delay(0.02)
        """
        if arm is not None:
            self.set_arm_angle_async(arm)
        if hand is not None:
            self.set_hand_angle_async(hand)
        # XY 双轴: 直接驱动一个 tick(与 goto_position_async 一致)
        if x is not None or y is not None:
            self.goto_position_async(x, y)

    def set_arm_pose(self, x=None, y=None, arm=None, hand=None):
        """
        设置机械臂的位姿(同步, 阻塞)。

        Args:
            x: 水平位置
            y: 竖直位置
            arm: 手臂角度，可以是字符串（"LEFT", "MID", "RIGHT"）或数字
            hand: 手部角度，可以是字符串（"UP", "MID", "DOWN"）或数字

        """
        self.goto_position(x, y)
        # time.sleep(0.2)
        if arm is not None:
            self.set_arm_angle(arm)
        if hand is not None:
            self.set_hand_angle(hand)

    # ==================== 便捷属性接口 ====================
    @property
    def y(self) -> float:
        """获取当前竖直位置（单位：mm）"""
        return self.y_get_position() * 1000.0

    @y.setter
    def y(self, mm: float):
        """设置目标竖直位置（单位：mm）"""
        self.move_y_position(mm / 1000.0)

    @property
    def x(self) -> float:
        """获取当前水平位置（单位：mm）"""
        return self.x_get_position() * 1000.0

    @x.setter
    def x(self, mm: float):
        """设置目标水平位置（单位：mm）"""
        self.move_x_position(mm / 1000.0)

    @property
    def angle(self) -> float:
        """获取手臂舵机当前角度"""
        return self._arm_angle_last if hasattr(self, "_arm_angle_last") else 0

    @angle.setter
    def angle(self, val: Union[str, int]):
        """设置手臂舵机角度"""
        self.set_arm_angle(val)

    @property
    def hand_angle(self) -> float:
        """获取手部舵机当前角度"""
        return self._hand_angle_last if hasattr(self, "_hand_angle_last") else 0

    @hand_angle.setter
    def hand_angle(self, val: Union[str, int]):
        """设置手部舵机角度"""
        self.set_hand_angle(val)


if __name__ == "__main__":
    arm = ArmController()
