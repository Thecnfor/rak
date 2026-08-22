#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import os
import sys
import threading
import time

from smartcar import Camera, Streamer, PID, logger
from smartcar.whalesbot.tools import get_yaml
from smartcar.whalesbot.vehicle import (
    ArmController,
    Beep,
    Key4Btn,
    MecanumDriver,
    ScreenShow,
    ServoPwm,
)
from smartcar.whalesbot.vehicle.base.controller_wrap import PoutD

from .motion import MotionMixin
from .perception import PerceptionMixin
from .global_pose import GlobalPose
from . import cfg as tasks_cfg


class MyCar(MotionMixin, PerceptionMixin, MecanumDriver):
    """
    智能车控制类

    该类继承自MecanumDriver，实现了智能车的完整控制功能，包括传感器初始化、PID控制、摄像头控制、
    目标检测、车道保持等功能。移动与感知方法分别由 MotionMixin / PerceptionMixin 提供。
    """

    STOP_PARAM: bool = True

    def __init__(self, stream=True):
        """
        初始化智能车

        初始化智能车的各个组件，包括底盘、传感器、摄像头、PID控制器等。

        参数:
            stream (bool): 是否启动 MJPEG 推流(Flask 服务 + 每帧 JPEG 编码)。
                比赛时不需要看画面可传 False(或设环境变量 SMARTCAR_NO_STREAM=1),
                省掉推流线程和每帧编码/叠加的 CPU; 检测/巡线推理线程不受影响。
        """
        # 调用继承的初始化
        start_time = time.time()
        super(MyCar, self).__init__()
        logger.info("my car init ok {}".format(time.time() - start_time))
        # 显示
        self.display = ScreenShow()

        # 推流可选: 不推流时 streamer 为 None, realtime 推流线程自动空转跳过
        if stream and os.environ.get("SMARTCAR_NO_STREAM", "") != "1":
            self.streamer = Streamer()
        else:
            self.streamer = None
            logger.info("推流已禁用(stream=False 或 SMARTCAR_NO_STREAM=1)")
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
        # 侧视(cam2: 检测推理+推流) + 前视(cam1: 巡线推理+描边推流) 共 4 个后台线程
        self.start_realtime_streams()
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

        # 全局坐标层: 维护"里程计系 -> 场地系"变换, reset_position 不丢全局位姿
        # (详见 tasks/tools/global_pose.py; 锚定用 set_field_origin)
        self.global_pose = GlobalPose(odom_getter=self.get_odometry)

        self.beep()

    def beep(self):
        """
        发出蜂鸣音

        控制蜂鸣器发出一声蜂鸣音，并等待0.2秒。
        """
        self.ring.rings()

    # ==================================================================
    # 全局坐标层接口(场地坐标系)
    # ==================================================================
    def set_field_origin(self, x=0.0, y=0.0, theta=0.0):
        """锚定场地坐标系: 声明"车此刻在场地 (x, y, theta)".

        典型用法: 把车摆到场地出发点并摆正朝向后, 调 set_field_origin(0, 0, 0),
        此后 get_global_pose() 即为场地坐标; 也可在识别到已知地标时用
        地标的真实场地坐标调用, 修正里程计漂移。
        """
        self.global_pose.anchor([x, y, theta])

    def get_global_pose(self) -> list:
        """读当前场地系位姿 [x, y, theta] (米/弧度). 不受 reset_position 影响."""
        return self.global_pose.to_global(self.get_odometry())

    def get_global_odometry_str(self) -> str:
        """场地系位姿可读串(x.x, y.y m / deg), 用于日志/屏幕显示."""
        gx, gy, gth = self.get_global_pose()
        return "global[{:+.2f},{:+.2f}m {:+.0f}deg]".format(
            gx, gy, math.degrees(gth)
        )

    def go_to_global_pose(
        self, target_global, max_velocities=None, tolerance=None, timeout=30.0
    ) -> bool:
        """按场地坐标闭环导航到 target_global [x, y, theta](米/弧度).

        换算回里程计系后走底层 move_to_position(带 PID 闭环/最短转向),
        返回是否收敛。中途 reset_position 也没关系(每次迭代重读变换)。
        """
        target_odom = self.global_pose.to_odom(
            [float(target_global[0]), float(target_global[1]), float(target_global[2])]
        )
        try:
            self.move_to_position(
                target_odom,
                None,
                max_velocities or [0.2, 0.2, math.pi / 3],
                tolerance or [0.004, 0.004, 0.02],
                timeout,
            )
        except Exception as e:
            logger.warning(f"go_to_global_pose 异常: {e}")
            return False
        return True

    def reset_position(self, x=0, y=0, z=0.0, distance=0):
        """覆写底盘 reset_position: 先保全全局坐标再清零里程计.

        车没动、只是坐标系被清, 所以全局位姿必须连续 —— 用 reset 前的
        位姿算出全局值, 再以 reset 后的新坐标系重锚回去。此后
        get_global_pose() 返回值与 reset 前一致(不跳变)。
        """
        old_odom = self.get_odometry()
        super().reset_position(x, y, z, distance)
        self.global_pose.on_odom_reset(old_odom, [float(x), float(y), float(z)])


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

    def shooting(self, pulse_seconds=0.28):
        """击发一发子弹 (电磁式: 继电器 → 击发电磁铁)。

        参数:
            pulse_seconds: 击发线圈通电时长 (s)。
                电磁枪通常 50~300ms, 默认 100ms (与 test_shoot.py 校准一致)。
                过短 = 撞针冲不到位; 过长 = 线圈过热且不增加威力。
        """
        import logging as _log

        self.beep()  # 击发前蜂鸣, 提示即将射击 (方便听觉对齐)
        time.sleep(0.1)

        t0 = time.time()
        self.shoot.set(1)  # 继电器吸合 → 电磁铁得电
        time.sleep(max(pulse_seconds, 0.001))
        self.shoot.set(0)  # 继电器断开 → 复位
        elapsed = time.time() - t0

        time.sleep(0.1)
        self.beep()  # 击发后蜂鸣, 提示完成
        logger.info(
            "shooting() done: pulse={:.0f}ms, actual={:.0f}ms".format(
                pulse_seconds * 1000, elapsed * 1000
            )
        )
        print(
            "[shooting] 脉冲 {:d}ms → 实际通电 {:.0f}ms".format(
                int(pulse_seconds * 1000), elapsed * 1000
            )
        )

    def car_pid_init(self, cfg):
        """
        初始化PID控制器

        根据配置初始化车道保持和目标检测的PID控制器。

        参数:
            cfg: 配置字典，包含PID控制器的配置信息
        """
        self.lane_pid_y = PID(**cfg["lane_pid"]["cfg_pid_y"])
        # 转向 PID: 参数统一走 tasks/tools/cfg.py (实车标定), 覆盖 yaml 值
        self.lane_pid_angle = PID(
            Kp=tasks_cfg.PID_ANGLE_KP,
            Ki=tasks_cfg.PID_ANGLE_KI,
            Kd=tasks_cfg.PID_ANGLE_KD,
            setpoint=0,
            output_limits=tasks_cfg.PID_ANGLE_LIMITS,
        )

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

        采用异步读按键 + 0.2s 轮询节奏: 每次只发一帧异步读命令(不等应答),
        不阻塞串口总线, 与其他设备(机械臂/底盘/舵机)共享总线时冲突最小。
        """
        while True:
            if self._end_flag:
                return
            if not self._stop_flag:
                self.key.get_key_async(callback=self._on_key)
                # print(key_val)
            time.sleep(0.2)

    def _on_key(self, key_val):
        """按键异步回调(在串口读线程中执行), 键值 3 表示急停。"""
        if key_val == 3:
            self._stop_flag = True

    def close(self):
        """
        关闭方法

        关闭顺序: 先停推流线程 (streamer), 再释放 VideoCapture。
        避免后台线程在已释放的 cv2 对象上读帧, 触发 C++ 层 SIGABRT。
        """
        self._stop_flag = False
        self._end_flag = True
        try:
            self.thread_key.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.streamer.stop()
        except Exception:
            pass
        try:
            self.cap_front.close()
        except Exception:
            pass
        try:
            self.cap_side.close()
        except Exception:
            pass
        # self.grap_cam.close()


def create_car(reset=True, comp_mode=False, stream=True):
    """
    参数:
        reset (bool): True 时执行蜂鸣提示 + 机械臂复位 + 里程计清零。
        comp_mode (bool): 是否进入比赛模式。True 时按键交给任务编排器
            (Orchestrator) 统一处理（4=一键启动/重来, 1=跳过, 3=急停），
            因此关闭 MyCar 内置的按键线程，避免双线程同时读按键造成串口冲突；
            任务编排与其他功能保持不变。
        stream (bool): 是否启动 MJPEG 推流(透传给 MyCar, 默认 True)。
    """
    car = MyCar(stream=stream)
    car.STOP_PARAM = False
    if comp_mode:
        # 比赛模式：禁用 MyCar 内置按键线程（按键统一由 Orchestrator 接管）
        car._end_flag = True
        car.thread_key.join()
    if reset:
        car.beep()
        car.arm.reset_position()
        car.reset_position()
    return car
