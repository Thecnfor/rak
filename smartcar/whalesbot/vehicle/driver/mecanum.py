#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Mecanum轮底盘控制模块

此模块实现了Mecanum轮底盘的完整控制功能，包括：
- 运动学计算（正逆解）
- 速度控制
- 里程计更新
- 坐标系转换
- PID位置控制

所有功能均内置，不依赖外部文件，可直接执行或集成到更大的系统中。
"""

import math
import threading
import numpy as np
import yaml
import os
import time
import sys


class _Offset:
    """偏移量对象，支持 += 操作"""
    def __init__(self, driver, axis: str, unit: str = 'mm'):
        self._driver = driver
        self._axis = axis
        self._unit = unit

    def __iadd__(self, delta):
        self._driver.offset_by(self._make_offset(delta))
        return self

    def __isub__(self, delta):
        self._driver.offset_by(self._make_offset(-delta))
        return self

    def _make_offset(self, delta):
        return [delta, 0, 0] if self._axis == 'x' else ([0, delta, 0] if self._axis == 'y' else [0, 0, delta])


class _OffsetGroup:
    """偏移量组，管理 x/y/z 三个轴的偏移"""
    def __init__(self, driver):
        # 使用对象属性存储，避免被覆盖
        self._x = _Offset(driver, 'x')
        self._y = _Offset(driver, 'y')
        self._z = _Offset(driver, 'z')

    def __getattr__(self, name):
        if name == 'x':
            return self._x
        if name == 'y':
            return self._y
        if name == 'z':
            return self._z
        raise AttributeError(f"'{type(self).__name__}' has no attribute '{name}'")

    def __setattr__(self, name, value):
        if name in ('_x', '_y', '_z', '_driver'):
            super().__setattr__(name, value)
        elif isinstance(value, _Offset):
            # 允许 +=/-= 操作返回的 _Offset 对象重新赋值
            super().__setattr__(name, value)
        else:
            raise AttributeError(f"'{type(self).__name__}' does not allow direct assignment to '{name}' - use += or -= instead")


# 导入自定义log模块
from ...tools.log_wrap import logger
from ..base.controller_wrap import WheelWrap
from ...tools import PID


class Odometry:
    """
    里程计基础类

    用于计算和更新车辆的位姿信息，以及处理坐标系转换
    """

    def __init__(self):
        """
        初始化里程计

        初始位姿为 [0.0, 0.0, 0.0]，初始速度为 [0.0, 0.0, 0.0]
        """
        # x, y, theta
        self.position = np.array(
            [0.0, 0.0, 0.0], dtype=np.float32
        )  # 世界坐标系下的位姿
        # 速度
        self.velocity = np.array(
            [0.0, 0.0, 0.0], dtype=np.float32
        )  # 世界坐标系下的速度
        # 车子整体前进的路程变量
        self.distance = 0.0

    def update(self, d_vector):
        """
        更新里程计数据

        参数:
            d_vector: 车辆坐标系下的位移向量 [dx, dy, dtheta]
        """
        # 位置变化矩阵
        z_angle = self.position[2]
        d_pose_transform = np.array(
            [
                [math.cos(z_angle), math.sin(z_angle)],
                [-math.sin(z_angle), math.cos(z_angle)],
            ],
            dtype=np.float32,
        )
        # 车子坐标变化转为世界坐标变化
        d_pose_xy = np.dot(d_vector[:2], d_pose_transform)
        # 更新路程
        self.distance += float(np.sum(d_vector[:2] ** 2, keepdims=True) ** 0.5)
        # 增加角度变化量
        d_pose = np.append(d_pose_xy, values=d_vector[2]).astype(np.float64)
        # 更新世界坐标位置
        self.position += d_pose

    def reset(self, x=None, y=None, z=None, distance=None):
        """
        重置位姿为指定位置, 默认保持原有值不变
        
        参数:
            x: x轴位置，为 None 时保持原值
            y: y轴位置，为 None 时保持原值
            z: theta角度，为 None 时保持原值
            distance: 距离参数，为 None 时保持原值
        """
        # 取出当前的 x, y, z 值
        current_x, current_y, current_z = self.position
        current_distance = self.distance
        
        # 逐个判断：传入非 None 则更新，否则用原值
        new_x = x if x is not None else current_x
        new_y = y if y is not None else current_y
        new_z = z if z is not None else current_z
        # 赋值新位置
        self.distance = distance if distance is not None else current_distance
        self.position = np.array([new_x, new_y, new_z], dtype=np.float32)
        
            

    def world_to_car_velocity(self, vel_world, angle_car):
        """
        世界坐标系速度转换为车辆坐标系速度

        参数:
            vel_world: 世界坐标系下的速度向量 [vx, vy, vtheta]
            angle_car: 车辆当前角度（弧度）

        返回:
            numpy.ndarray: 车辆坐标系下的速度向量 [vx, vy, vtheta]
        """
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # 世界坐标系到车辆坐标系的转换矩阵
        transform = np.array([[cos_car, -sin_car, 0], [sin_car, cos_car, 0], [0, 0, 1]])
        vel_car = np.array(vel_world).dot(transform)
        return vel_car

    def car_to_world_velocity(self, vel_car, angle_car):
        """
        车辆坐标系速度转换为世界坐标系速度

        参数:
            vel_car: 车辆坐标系下的速度向量 [vx, vy, vtheta]
            angle_car: 车辆当前角度（弧度）

        返回:
            numpy.ndarray: 世界坐标系下的速度向量 [vx, vy, vtheta]
        """
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # 车辆坐标系到世界坐标系的转换矩阵
        transform = np.array([[cos_car, -sin_car, 0], [sin_car, cos_car, 0], [0, 0, 1]])
        vel_world = np.array(vel_car).dot(transform)
        return vel_world


class MecanumChassis:
    """
    麦克纳姆轮底盘类

    轮子布局：
        [2]**[1]
        ********
        ********
        [3]**[4]

    从上方往下方看是x形排布, 轮子接触地面是O形排布
    轮子速度定义为轮子顺时针转动为正
    """

    def __init__(self, track=0.30, wheel_base=0.28, wheel_radius=0.03):
        """
        初始化麦克纳姆轮底盘

        参数:
            track: 轮距
            wheel_base: 轴距
            wheel_radius: 轮子半径
        """
        # 初始化里程计
        self.odometry = Odometry()
        # 轮距 轴距
        self.half_wheel_base = wheel_base / 2
        self.half_track = track / 2
        self.wheel_radius = wheel_radius
        self.init_parameters()

    def init_parameters(self):
        """
        初始化麦克纳姆轮底盘的转换矩阵
        """
        roller_angle = math.pi / 4 * 1.052
        tan_roller = math.tan(roller_angle)
        wheel_constant = self.half_track * tan_roller + self.half_wheel_base
        # 根据小车四轮运动计算小车运动，正解
        self.wheel_to_vehicle_matrix = np.array(
            [
                [1 / 4, 1 / 4 / tan_roller, 1 / wheel_constant / 4],
                [-1 / 4, 1 / 4 / tan_roller, 1 / wheel_constant / 4],
                [-1 / 4, -1 / 4 / tan_roller, 1 / wheel_constant / 4],
                [1 / 4, -1 / 4 / tan_roller, 1 / wheel_constant / 4],
            ]
        )

        # 根据小车运动计算小车四轮运动，逆解
        self.vehicle_to_wheel_matrix = np.array(
            [
                [1, -1, -1, 1],
                [tan_roller, tan_roller, -tan_roller, -tan_roller],
                [wheel_constant, wheel_constant, wheel_constant, wheel_constant],
            ]
        )

    def forward_kinematics(self, wheel_velocity: np.ndarray) -> np.ndarray:
        """
        正解计算：轮子速度 → 车辆速度

        参数:
            wheel_velocity: 轮子速度向量

        返回:
            numpy.ndarray: 车辆速度向量
        """
        return wheel_velocity @ self.wheel_to_vehicle_matrix

    def inverse_kinematics(self, car_velocity: np.ndarray) -> np.ndarray:
        """
        逆解计算：车辆速度 → 轮子速度

        参数:
            car_velocity: 车辆速度向量 [vx, vy, vtheta]

        返回:
            numpy.ndarray: 轮子速度向量
        """
        return car_velocity @ self.vehicle_to_wheel_matrix

    def calculate_wheel_velocities(self, x: float, y: float, z: float) -> np.ndarray:
        """
        计算轮子速度

        参数:
            x: x轴线速度
            y: y轴线速度
            z: theta角度

        返回:
            numpy.ndarray: 轮子线速度向量
        """
        # 计算小车每个轮子的线速度
        wheel_velocities = self.inverse_kinematics(np.array([x, y, z]))
        return wheel_velocities

    def update_odometry(self, wheel_displacements: np.ndarray):
        """
        更新里程计数据

        参数:
            wheel_displacements: 轮子位移向量
        """
        # 计算小车的位置变化
        car_displacement = self.forward_kinematics(wheel_displacements)
        # 更新小车的位姿
        self.odometry.update(car_displacement)


class MecanumDriver:
    """
    Mecanum轮底盘驱动类

    整合Mecanum轮底盘的控制功能，包括速度设置、位姿控制、里程计更新等

    用法示例:
        my_car.offset.x += 100  # 相对移动x方向100mm
        my_car.offset.y -= 50   # 相对移动y方向-50mm
        my_car.offset.z += 90  # 相对旋转90度
    """

    def __init__(self, config_file=None):
        """
        初始化MecanumDriver类

        参数:
            config_file: 配置文件路径，如果为None则使用默认配置
        """
        # 加载配置
        if config_file and os.path.exists(config_file):
            self.load_config(config_file)
        else:
            self.load_default_config()

        # 初始化底盘
        self.chassis = MecanumChassis(
            track=self.config["vehicle_cfg"]["MecanumChassis"]["size"]["track"],
            wheel_base=self.config["vehicle_cfg"]["MecanumChassis"]["size"][
                "wheel_base"
            ],
            wheel_radius=self.config["vehicle_cfg"]["wheel"]["raduis"],
        )

        # 初始化轮子
        motor_ports = self.config["vehicle_cfg"]["MecanumChassis"]["wheel"]["port_list"]
        wheel_params = self.config["vehicle_cfg"]["wheel"]
        self.wheels_chassis = WheelWrap(motor_ports, **wheel_params)

        # 初始化PID控制器
        self.pid_x = PID(**self.config["pid_vel_params"]["pid_x"])
        self.pid_y = PID(**self.config["pid_vel_params"]["pid_y"])
        self.pid_yaw = PID(**self.config["pid_vel_params"]["pid_yaw"])

        # 初始化线程锁，确保线程安全
        self._lock = threading.Lock()

        # 初始化偏移量接口
        self.offset = _OffsetGroup(self)
        self._stop_thread = False
        self.odometry_thread = threading.Thread(target=self.update_odometry_thread)
        self.odometry_thread.daemon = True  # 守护线程，程序结束时自动退出
        self.odometry_thread.start()

    def load_config(self, config_file):
        """
        加载配置文件

        参数:
            config_file: 配置文件路径
        """
        with open(config_file, "r", encoding="utf-8") as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

    def load_default_config(self):
        """
        加载默认配置
        """
        self.config = {
            "vehicle_cfg": {
                "chassis_type": "MecanumChassis",
                "wheel": {"motor_type": "motor_280", "raduis": 0.03},
                "MecanumChassis": {
                    "size": {"track": 0.30, "wheel_base": 0.28},  # 轮距  # 轴距
                    "wheel": {"port_list": [1, 2, 3, 4]},
                },
            },
            "pid_vel_params": {
                "pid_x": {
                    "Kp": 6,
                    "Ki": 0.3,
                    "Kd": 0.1,
                    "setpoint": 0,
                    "output_limits": [-0.6, 0.6],
                },
                "pid_y": {
                    "Kp": 8,
                    "Ki": 0.3,
                    "Kd": 0.1,
                    "setpoint": 0,
                    "output_limits": [-0.6, 0.6],
                },
                "pid_yaw": {
                    "Kp": 10,
                    "Ki": 0.2,
                    "Kd": 0.1,
                    "setpoint": 0,
                    "output_limits": [-1.5, 1.5],
                },
            },
        }

    def reset_position(self, x=0, y=0, z=0.0 ,distance = 0):
        """
        重置车辆位置为指定位置，默认为原点
        重置后，里程计数据将被清空
        参数:
            x: x轴位置
            y: y轴位置
            z: theta角度
            distance: 前进的距离
        """
        with self._lock:
            self.chassis.odometry.reset(x, y, z,distance)

    def world_to_car_velocity(self, vel_world, angle_car):
        """
        世界坐标系速度转换为车辆坐标系速度

        参数:
            vel_world: 世界坐标系下的速度向量 [vx, vy, vtheta]
            angle_car: 车辆当前角度（弧度）

        返回:
            numpy.ndarray: 车辆坐标系下的速度向量 [vx, vy, vtheta]
        """
        return self.chassis.odometry.world_to_car_velocity(vel_world, angle_car)

    def car_to_world_velocity(self, vel_car, angle_car):
        """
        车辆坐标系速度转换为世界坐标系速度

        参数:
            vel_car: 车辆坐标系下的速度向量 [vx, vy, vtheta]
            angle_car: 车辆当前角度（弧度）

        返回:
            numpy.ndarray: 世界坐标系下的速度向量 [vx, vy, vtheta]
        """
        return self.chassis.odometry.car_to_world_velocity(vel_car, angle_car)

    def set_velocity(self, x, y, z):
        """
        设置车辆速度

        参数:
            x: x轴线速度
            y: y轴线速度
            z: 角速度
        """
        # 根据速度计算四个轮子速度
        wheel_linear_velocities = self.chassis.calculate_wheel_velocities(x, y, z)
        # 设置轮子线速度
        self.wheels_chassis.set_linear(wheel_linear_velocities)

    def set_velocity_for_duration(self, x, y, z, duration=1.0):
        """
        在指定时间内保持速度

        参数:
            x: x轴线速度
            y: y轴线速度
            z: 角速度
            duration: 持续时间（秒）
        """
        start_time = time.time()
        while True:
            if time.time() - start_time > duration:
                break
            self.set_velocity(x, y, z)
            time.sleep(0.01)
        self.set_velocity(0, 0, 0)

    def update_odometry_thread(self):
        """
        里程计更新线程函数

        定期更新车辆位姿信息
        """
        previous_wheel_linear_velocities = np.array(self.wheels_chassis.get_linear())
        while True:
            if self._stop_thread:
                break
            current_wheel_linear_velocities = self.wheels_chassis.get_linear()
            # 获取每个轮子的位移
            wheel_linear_displacements = (
                current_wheel_linear_velocities - previous_wheel_linear_velocities
            )
            previous_wheel_linear_velocities = current_wheel_linear_velocities
            # 里程计根据轮子的位置变化更新，使用锁确保线程安全
            with self._lock:
                self.chassis.update_odometry(wheel_linear_displacements)
            time.sleep(0.05)

    def get_odometry(self,show_info=False) -> np.ndarray:
        """
        获取当前位姿

        返回:
            numpy.ndarray: 当前位姿 [x, y, theta]
        """
        with self._lock:
            if show_info:
                logger.info(f"当前位姿: [{self.chassis.odometry.position[0]:.4f}, {self.chassis.odometry.position[1]:.4f}, {self.chassis.odometry.position[2]:.4f}]")
            return self.chassis.odometry.position.copy()

    def get_distance(self,show_info=False) -> float:
        """
        获取行驶距离

        返回:
            float: 行驶距离
        """
        with self._lock:
            if show_info:
                logger.info(f"当前行驶距离: {self.chassis.odometry.distance:.4f}")
            return self.chassis.odometry.distance

    def stop(self):
        """
        停止车辆
        """
        self.set_velocity(0, 0, 0)

    def close(self):
        """
        关闭线程
        """
        self._stop_thread = True
        self.odometry_thread.join()

    def move_to_position(
        self,
        target_position,
        duration=None,
        max_velocities=(0.2, 0.2, math.pi / 3),
        tolerance=(0.004, 0.004, 0.02),
        timeout=30.0,  # 添加超时参数
    ):
        """
        通过PID控制移动到指定位姿

        参数:
            target_position: 目标位置 [x, y, theta]
            duration: 预计运动时长，设置后将自动计算速度上限
            max_velocities: 速度上限 [x轴速度, y轴速度, 角速度]
            tolerance: 位置误差阈值 [x误差, y误差, 角度误差]
            timeout: 超时时间（秒），超过此时间将停止尝试
        """

        with self._lock:
            current_position = self.chassis.odometry.position.copy()

        if duration is not None:
            computed_limits = (
                np.abs(np.array(target_position) - current_position)
            ) / duration
            max_velocities = computed_limits

        self.pid_x.setpoint = target_position[0]
        self.pid_x.output_limits = (-max_velocities[0], max_velocities[0])
        self.pid_y.setpoint = target_position[1]
        self.pid_y.output_limits = (-max_velocities[1], max_velocities[1])

        tolerance = np.array(tolerance)

        self.pid_yaw.setpoint = target_position[2]
        self.pid_yaw.output_limits = (-max_velocities[2], max_velocities[2])
        consecutive_within_threshold = 0
        start_time = time.time()
        iteration_count = 0
        max_iterations = 1000  # 最大迭代次数

        while True:
            # 检查超时
            if time.time() - start_time > timeout:
                break

            # 检查最大迭代次数
            iteration_count += 1
            if iteration_count > max_iterations:
                break

            with self._lock:
                current_position = self.chassis.odometry.position.copy()

            error = np.abs(current_position - target_position)
            error_within_threshold = error < tolerance

            if error_within_threshold.all():
                consecutive_within_threshold += 1
                if consecutive_within_threshold > 20:
                    break
            else:
                consecutive_within_threshold = 0

            velocity_x_pid = self.pid_x(current_position[0])
            velocity_y_pid = self.pid_y(current_position[1])
            angular_velocity_pid = self.pid_yaw(current_position[2])
            # 世界坐标速度转换车子坐标速度
            velocity_output = self.world_to_car_velocity(
                [velocity_x_pid, velocity_y_pid, angular_velocity_pid],
                current_position[2],
            )
            self.set_velocity(*velocity_output)
            time.sleep(0.01)  # 添加小延迟，避免CPU占用过高
        self.set_velocity(0, 0, 0)

    def move_for(
        self,
        position_offset,
        duration=None,
        max_velocities=None,
        tolerance=None,
    ):
        """
        基于底盘当前位置，叠加一个相对偏移量，计算并移动到目标绝对位置

        参数：
            position_offset: 相对位置偏移量 [x偏移, y偏移, 角度偏移]
            duration: 预计运动时长，设置后将自动计算速度上限，默认不启用
            max_velocities: 速度上限 [x轴速度, y轴速度, 角速度]，默认值 [0.2, 0.2, π/3]
            tolerance: 位置误差阈值 [x误差, y误差, 角度误差]，单位：米、弧度，默认值 [0.002, 0.002, 0.02]
        """
        if max_velocities is None:
            max_velocities = [0.2, 0.2, math.pi / 3]
        if tolerance is None:
            tolerance = [0.002, 0.002, 0.02]
        with self._lock:
            current_position = self.chassis.odometry.position.copy()

        target_position = [0, 0, 0]
        target_position[0] = (
            current_position[0]
            + position_offset[0] * math.cos(current_position[2])
            - position_offset[1] * math.sin(current_position[2])
        )
        target_position[1] = (
            current_position[1]
            + position_offset[1] * math.cos(current_position[2])
            + position_offset[0] * math.sin(current_position[2])
        )
        target_position[2] = current_position[2] + position_offset[2]
        self.move_to_position(target_position, duration, max_velocities, tolerance)

    def offset_by(self, position_offset, duration=None, max_velocities=None, tolerance=None):
        """
        相对移动指定偏移量（单位：x/y为mm，z为度）

        参数:
            position_offset: 相对位置偏移量 [x偏移(mm), y偏移(mm), 角度偏移(度)]
            duration: 预计运动时长（秒）
            max_velocities: 速度上限 [x轴速度, y轴速度, 角速度]
            tolerance: 位置误差阈值 [x误差, y误差, 角度误差]
        """
        offset = [
            position_offset[0] / 1000.0,
            position_offset[1] / 1000.0,
            math.radians(position_offset[2]),
        ]
        self.move_for(offset, duration, max_velocities, tolerance)

    # ==================== 便捷属性接口 ====================
    @property
    def x(self) -> float:
        """获取当前x位置（单位：mm）"""
        with self._lock:
            return self.chassis.odometry.position[0] * 1000.0

    def move_x(self, mm: float):
        """相对移动x（单位：mm）"""
        self.offset_by([mm, 0, 0])

    def move_y(self, mm: float):
        """相对移动y（单位：mm）"""
        self.offset_by([0, mm, 0])

    def move_z(self, degrees: float):
        """相对旋转角度（单位：度）"""
        self.offset_by([0, 0, degrees])


if __name__ == "__main__":
    """
    测试代码
    """
    np.set_printoptions(precision=4, suppress=True)
    print("Mecanum轮底盘控制测试")

    try:
        # 初始化MecanumDriver
        config_path = os.path.join(os.path.dirname(__file__), "cfg_vehicle.yaml")
        driver = MecanumDriver(config_path)

        # 测试初始化
        print("\n初始化成功")
        print("当前位姿:", driver.get_odometry())

        # # 测试速度控制（简短测试）
        # print("\n测试速度控制...")
        # print("前进0.5秒")
        # driver.set_velocity_for_duration(0.2, 0, 0, 0.5)
        # time.sleep(1)
        # driver.get_odometry(show_info=True)

        # print("旋转0.5秒")
        # driver.set_velocity_for_duration(0, 0, math.pi / 2, 0.5)
        # time.sleep(1)
        # driver.get_odometry(show_info=True)


        driver.move_to_position([0.5, 0.2, 0])
        time.sleep(1)
        driver.get_odometry(show_info=True)

        driver.move_to_position([0, -0.5, math.pi / 2])
        time.sleep(1)
        driver.get_odometry(show_info=True)

        driver.get_distance(show_info=True)

        # 停止并关闭
        driver.stop()
        driver.close()
        print("\n测试完成")
    except Exception as e:
        print(f"错误: {e}")
        import traceback

        traceback.print_exc()
