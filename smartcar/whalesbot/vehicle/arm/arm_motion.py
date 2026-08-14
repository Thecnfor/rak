#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""机械臂 XY 轴运动层: ArmMotion mixin。

从原 arm_base.py 拆出, 包含水平(X)/竖直(Y)双轴的全部运动能力:
  - 参数初始化与位姿读取
  - PID 同步移动(move_x/y_position / goto_position / go_for)
  - 异步 tick 移动(tick_x/y_moveto / goto_position_async, 配合异步串口引擎)

本 mixin 的方法运行时由 ArmController 组合提供依赖属性(motor_x/motor_y/PID 等),
不直接实例化; 归入手部/姿态的方法保留在 arm_base.ArmController。
"""
import sys
import os
import time

# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__)))
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from ...tools import get_yaml, limit_val, CountRecord, PID, logger
from .. import MotorWrap, StepperWrap, AnalogInput

# 位置误差阈值(判定到达目标) - 硬件标定值
POSITION_ERROR_THRESHOLD = 4e-4
# 停止检测阈值 - 硬件标定值
STOP_CHECK_THRESHOLD = 1e-10
# 复位超时(秒): 机械臂归位卡死时兜底, 避免无限循环挂起整个流程
RESET_TIMEOUT = 10.0


class ArmMotion:
    """机械臂 XY 双轴运动能力 mixin。"""

    # ==================== 竖直(Y)轴 ====================
    def y_params_init(self, motor, limit_port, pid, threshold):
        """
        初始化竖直方向电机参数

        Args:
            motor: 电机配置
            limit_port: 限位传感器端口
            pid: PID参数
            threshold: 位置阈值
        """
        self.motor_y = StepperWrap(**motor)
        self.y_limit_sensor = AnalogInput(limit_port)

        self.y_pose_start = self.motor_y.get_dis()
        self.y_pose_now = 0
        self.y_pid = PID(**pid)
        self.y_velocity_limit = pid['output_limits']
        self.y_distance_change = 0
        self.y_threshold = threshold  # 竖直位置阈值
        self.y_pose_last = 0

        self.y_pid_flag = CountRecord(5)
        self.y_stop_flag = CountRecord(10)

    def y_reset_check(self):
        """
        检查竖直方向是否到达限位

        Returns:
            bool: 是否到达限位
        """
        return self.y_limit_sensor.read() > 1000  # 磁敏传感器的值大于1000时, 则认为到达限位位置

    def y_stop_check(self):
        """
        检查竖直方向是否停止

        Returns:
            bool: 是否停止
        """
        return self.y_stop_flag(
            abs(self.y_distance_change) < STOP_CHECK_THRESHOLD
        )

    def y_get_position(self):
        self.y_pose_now = (
            self.motor_y.get_dis() - self.y_pose_start
        )
        return self.y_pose_now

    def y_pid_moveto(self, target_pose):
        """
        使用PID控制竖直方向移动

        Args:
            target_pose: 目标位置 (单位: m)

        Returns:
            bool: 是否到达目标位置
        """
        # 记录当前位置, 并更新上次的位置
        self.y_pose_now = (
            self.motor_y.get_dis() - self.y_pose_start
        )
        self.y_distance_change = (
            self.y_pose_now - self.y_pose_last
        )
        self.y_pose_last = self.y_pose_now

        error = target_pose - self.y_pose_now
        velocity = self.y_pid(self.y_pose_now)

        self.y_speed(velocity)

        if self.y_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            return True
        else:
            return False

    def reset_y(self):
        """
        重置竖直方向位置
        """
        self.y_pid.setpoint = -0.25
        start_time = time.time()
        while True:
            if time.time() - start_time > RESET_TIMEOUT:
                logger.warning("机械臂 Y 轴复位超时, 跳过(请检查电机/限位传感器)")
                break
            if self.y_pid_moveto(-0.25):
                break
            if self.y_reset_check():
                self.y_pose_start = self.motor_y.get_dis()
                self.y_pose_now = 0
                break
        self.y_speed(0)

    def move_y_position(self, target):
        """
        移动竖直方向指定距离

        Args:
            target: 目标位置
        """
        self.y_pid.setpoint = target
        while True:
            if self.y_pid_moveto(target):
                logger.info(f"移动到高度{target}")
                break
            if self.y_stop_check():
                logger.info(f"移到高度{target}过程中检测到停止")
                break
        self.y_speed(0)

    def y_speed(self, velocity):
        """
        设置竖直方向速度

        Args:
            velocity: 速度值
        """
        velocity = limit_val(velocity, *self.y_velocity_limit)
        self.motor_y.set_velocity(velocity)

    def tick_y_moveto(self, target):
        """单步驱动竖直轴向 target 移动, 立即返回是否到位。同 tick_x_moveto。"""
        self.y_pid.setpoint = target
        self.y_pose_now = self.motor_y.get_dis() - self.y_pose_start
        error = target - self.y_pose_now
        if self.y_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            self.y_speed(0)
            return True
        velocity = self.y_pid(self.y_pose_now)
        self.y_speed_async(velocity)
        return False

    def y_speed_async(self, velocity):
        """异步设置竖直轴速度(发命令不等应答), 减少总线占用。"""
        velocity = limit_val(velocity, *self.y_velocity_limit)
        try:
            stepper = getattr(self.motor_y, 'stepper', None)
            if stepper is not None and hasattr(stepper, 'set_async'):
                pwm = int(velocity * self.motor_y.dis2rad * self.motor_y.rad2pwm * self.motor_y.reverse)
                stepper.set_async(pwm)
            else:
                self.y_speed(velocity)
        except Exception as e:
            logger.warning("y_speed_async 失败, 回退同步: {}".format(e))
            self.y_speed(velocity)

    # ==================== 水平(X)轴 ====================
    def x_params_init(self, motor, pid, threshold):
        """
        初始化水平方向电机参数

        Args:
            motor: 电机配置
            pid: PID参数
            threshold: 位置阈值
        """
        # 定义水平移动电机,PID参数
        self.motor_x = MotorWrap(**motor)
        self.x_pid = PID(**pid)
        self.x_velocity_limit = pid['output_limits']
        self.x_pose_start = self.motor_x.get_dis()
        self.x_pose_now = 0
        self.x_threshold = threshold
        self.x_pose_last = 0

        self.x_distance_change = 0

        self.x_stop_flag = CountRecord(10)
        self.x_pid_flag = CountRecord(5)

    def x_stop_check(self):
        """
        检查水平方向是否停止

        Returns:
            bool: 是否停止
        """
        return self.x_stop_flag(
            abs(self.x_distance_change) < STOP_CHECK_THRESHOLD
        )

    def x_get_position(self):
        self.x_pose_now = self.motor_x.get_dis() - self.x_pose_start
        return self.x_pose_now

    def x_pid_moveto(self, target_pose):
        """
        使用PID控制水平方向移动

        Args:
            target_pose: 目标位置

        Returns:
            bool: 是否到达目标位置
        """
        self.x_pose_now = (
            self.motor_x.get_dis() - self.x_pose_start
        )
        self.x_distance_change = (
            self.x_pose_now - self.x_pose_last
        )
        self.x_pose_last = self.x_pose_now
        error = target_pose - self.x_pose_now

        velocity = self.x_pid(self.x_pose_now)

        self.x_speed(velocity)

        if self.x_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            return True
        else:
            return False

    def move_x_position(self, target, out_time=6.0):
        """
        移动水平方向指定位置

        Args:
            target: 目标位置
        """
        end_time = time.time() + out_time
        self.x_pid.setpoint = target
        while True:
            if time.time() > end_time:
                break
            if self.x_pid_moveto(target):
                break
            if self.x_stop_check():
                dis = self.motor_x.get_dis()
                if dis < 0.15:
                    self.x_pose_start = dis
                else:
                    self.x_pose_start = dis - 0.31
                break
            time.sleep(0.05)
        self.x_speed(0)

    def x_speed(self, velocity):
        """
        设置水平方向速度

        Args:
            velocity: 速度值
        """
        velocity = limit_val(velocity, *self.x_velocity_limit)
        self.motor_x.set_linear(velocity)

    def tick_x_moveto(self, target):
        """单步驱动水平轴向 target 移动, 立即返回是否到位。
        每次调用只发一条 PID 速度命令(异步, 不等应答), 由外部循环控制节奏。
        返回: True 到位 / False 未到位。
        """
        self.x_pid.setpoint = target
        self.x_pose_now = self.motor_x.get_dis() - self.x_pose_start
        error = target - self.x_pose_now
        if self.x_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            self.x_speed(0)
            return True
        velocity = self.x_pid(self.x_pose_now)
        self.x_speed_async(velocity)
        return False

    def x_speed_async(self, velocity):
        """异步设置水平轴速度(发命令不等应答), 减少总线占用。"""
        velocity = limit_val(velocity, *self.x_velocity_limit)
        try:
            # motor_x 为 MotorWrap, 其 .motor 为 Motor(内部有 motor_2 = Motor_2, 支持异步设速)
            motor2 = getattr(getattr(self.motor_x, 'motor', None), 'motor_2', None)
            if motor2 is not None and hasattr(motor2, 'set_speed_async'):
                angular = velocity * self.motor_x.dis2rad
                sp_virtual = int(self.motor_x.motor.rad2virtual * angular)
                motor2.set_speed_async(sp_virtual)
            else:
                self.x_speed(velocity)
        except Exception as e:
            logger.warning("x_speed_async 失败, 回退同步: {}".format(e))
            self.x_speed(velocity)

    def reset_x(self):
        """
        重置水平方向位置
        """
        target = -0.33
        self.x_pid.output_limits = (-0.06, 0.06)
        self.x_pid.setpoint = target
        start_time = time.time()
        while True:
            if time.time() - start_time > RESET_TIMEOUT:
                logger.warning("机械臂 X 轴复位超时, 跳过(请检查电机/接线)")
                break
            if self.x_pid_moveto(target):
                break
            if self.x_stop_check():
                self.x_pose_start = self.motor_x.get_dis()
                self.x_pose_now = 0
                self.x_pose_last = 0
                break
        self.x_speed(0)

    # ==================== 双轴运动编排 ====================
    def go_for(self, x_offset, y_offset, time_run=None, speed=[0.15, 0.04]):
        """
        移动机械臂到当前位置的相对量

        Args:
            x_offset: 水平偏移
            y_offset: 竖直偏移
            time_run: 运行时间
            speed: 速度 [水平速度, 竖直速度]
        """
        x_pos = self.x_pose_now + x_offset
        y_pos = self.y_pose_now + y_offset
        self.goto_position(x_pos, y_pos, time_run, speed)

    def goto_position(self, x=None, y=None, time_run=None, speed=[0.15, 0.04]):
        """
        移动到指定机械臂位置

        Args:
            x: 水平位置
            y: 竖直位置
            time_run: 运行时间
            speed: 速度 [水平速度, 竖直速度]
        """

        # 控制上下限
        x_pos = limit_val(
            x,
            self.x_threshold[0],
            self.x_threshold[1]
        )
        y_pos = limit_val(
            y,
            self.y_threshold[0],
            self.y_threshold[1]
        )

        # 获取结束时间和对应速度
        time_start = time.time()
        if time_run is not None:
            assert isinstance(time_run, (int, float)), "Time must be a number"
            # 根据时间求速度
            time_end = time_start + time_run
            y_time = time_run
            x_time = time_run
        elif speed is not None:
            # 根据速度求时间
            if isinstance(speed, (int, float)):
                speed_x = speed
                speed_y = speed
            elif isinstance(speed, (list, tuple)):
                speed_x = speed[0]
                speed_y = speed[1]
            else:
                logger.error("Invalid speed argument")
                return
            x_time = abs(
                x_pos - self.x_pose_now
            ) / speed_x
            y_time = abs(
                y_pos - self.y_pose_now
            ) / speed_y
            time_run = max(x_time, y_time)
        else:
            logger.error("Either time_run or speed must be provided")
            return
        # 超时时间
        time_end = time_start + time_run

        # 定义结束标志和到达位置标记量
        if y is None:
            y_flag = True
        else:
            y_flag = False

        if x is None:
            x_flag = True
        else:
            x_flag = False

        # 获取对应的速度和pid位置
        if y_time < 0.1:
            speed_y = 0.1
            y_flag = True
        else:
            speed_y = abs(
                y_pos - self.y_pose_now
            ) / y_time

        self.y_pid.setpoint = y_pos
        self.y_pid.output_limits = (-speed_y, speed_y)

        if x_time < 0.1:
            speed_x = 0.1
            x_flag = True
        else:
            speed_x = abs(
                x_pos - self.x_pose_now
            ) / x_time

        self.x_pid.setpoint = x_pos
        self.x_pid.output_limits = (
            -speed_x, speed_x
        )

        # 开始移动前, 位置信息定义, 如果中间中断此时位置信息无用
        self.save_config(pose_enable=False)

        while True:
            # 到达结束标志结束
            if y_flag and x_flag:
                break
            # 获取剩余时间
            time_remain = time_end - time.time()
            # 超时处理
            if time_remain < -3:
                logger.warning("Timeout")
                # 超时停止
                self.x_speed(0)
                self.y_speed(0)
                break
            if not y_flag:
                if self.y_pid_moveto(y_pos):
                    self.y_speed(0)
                    y_flag = True

                # 重置初始化位置
                if self.y_reset_check():
                    if self.y_pid.setpoint <= self.y_pose_now:
                        y_flag = True
                        self.y_speed(0)
                    self.y_pose_start = self.motor_y.get_dis()
                    self.y_pose_now = 0
                    self.save_config()

            if not x_flag:
                if self.x_pid_moveto(x_pos):
                    self.x_speed(0)
                    x_flag = True

        self.save_config()

    # ==================== 异步 tick 移动(配合异步串口引擎, 不阻塞主流程) ====================
    def goto_position_async(self, x=None, y=None, time_run=None, speed=[0.15, 0.04], tick_interval=0.02):
        """非阻塞版 goto_position: 由调用方在事件循环/主循环中周期性驱动。
        每次调用执行一个 tick(双轴各发一条异步速度命令), 全部到位返回 True。
        保持与 goto_position 相同的目标/限位/速度语义, 但单次调用立即返回, 不独占总线。

        用法:
            while not car.arm.goto_position_async(x, y):
                time.sleep(0.02)   # 期间可穿插底盘/感知命令
        """
        # 首次调用时初始化目标与速度(与 goto_position 一致)
        if not hasattr(self, '_async_plan') or self._async_plan is None:
            if self._init_async_plan(x, y, time_run, speed) is None:
                # 参数非法, 直接返回视为完成
                return True
        plan = self._async_plan
        # 双轴各驱动一 tick
        done_x = True if plan['x'] is None else self.tick_x_moveto(plan['x'])
        done_y = True if plan['y'] is None else self.tick_y_moveto(plan['y'])
        # 超时兜底
        if time.time() - plan['t0'] > plan['timeout']:
            self.x_speed(0)
            self.y_speed(0)
            self._async_plan = None
            logger.warning("goto_position_async 超时")
            return True
        if done_x and done_y:
            self.x_speed(0)
            self.y_speed(0)
            self._async_plan = None
            self.save_config()
            return True
        return False

    def _init_async_plan(self, x, y, time_run, speed):
        """初始化异步移动计划(目标/速度/超时), 与 goto_position 的限位与速度逻辑一致。"""
        x_pos = limit_val(x, self.x_threshold[0], self.x_threshold[1]) if x is not None else None
        y_pos = limit_val(y, self.y_threshold[0], self.y_threshold[1]) if y is not None else None
        if time_run is not None:
            assert isinstance(time_run, (int, float)), "Time must be a number"
            x_time = y_time = time_run
        elif speed is not None:
            if isinstance(speed, (int, float)):
                speed_x = speed
                speed_y = speed
            else:
                speed_x, speed_y = speed[0], speed[1]
            x_time = 0.0 if x_pos is None else abs(x_pos - self.x_pose_now) / speed_x
            y_time = 0.0 if y_pos is None else abs(y_pos - self.y_pose_now) / speed_y
        else:
            logger.error("Either time_run or speed must be provided")
            return None
        time_run_ = max(x_time, y_time)

        # 速度与 PID 限幅(与 goto_position 一致: output_limits = ±速度)
        if y_time < 0.1:
            speed_y = 0.1
        else:
            speed_y = 0.0 if y_pos is None else abs(y_pos - self.y_pose_now) / y_time
        if x_time < 0.1:
            speed_x = 0.1
        else:
            speed_x = 0.0 if x_pos is None else abs(x_pos - self.x_pose_now) / x_time

        if y_pos is not None:
            self.y_pid.setpoint = y_pos
            self.y_pid.output_limits = (-speed_y, speed_y)
        if x_pos is not None:
            self.x_pid.setpoint = x_pos
            self.x_pid.output_limits = (-speed_x, speed_x)

        self._async_plan = {
            'x': x_pos, 'y': y_pos,
            't0': time.time(), 'timeout': time_run_ + 3.0,
        }

    def cancel_async_move(self):
        """取消进行中的异步移动, 停止双轴。"""
        self.x_speed(0)
        self.y_speed(0)
        self._async_plan = None

    def set_position_start(self, y_position):
        """
        设置起始位置

        Args:
            y_position: 竖直位置
        """
        self.y_pose_start = self.y_pose_now
        self.x_pose_start = self.x_pose_now
        self.save_config()
