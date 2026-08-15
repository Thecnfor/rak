#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import os, math
import time
import numpy as np
import sys

# 添加上本地目录
dir_this = os.path.abspath(os.path.dirname(__file__))
sys.path.append(dir_this)
dir_root = os.path.abspath(os.path.join(dir_this, "..", ".."))
sys.path.append(dir_root)
# mc601
from .mc601_ctl2 import *

# mc602
from .mc602_ctl2 import *
from ...tools import PID, CountRecord

# 添加上两层目录

# 导入自定义log模块
from ...tools import logger

# logger.info("start time:{}".format(time.time()))

# 导入自定义log模块
serial_wrap = serial_wrap


# 获取devid
def get_devid():
    dev_name = serial_wrap.dev.name
    logger.info("dev_name:{}".format(dev_name))
    if "mc601" in dev_name:
        return 0
    elif "mc602" in dev_name:
        return 1


ctl_id = get_devid()


def limit_val(val, min_val, max_val):
    return max(min(val, max_val), min_val)


class MotorConvert:
    def __init__(self, perimeter=None) -> None:
        # 编码器一圈12栅格编码值48 , 减速比(28/11)^4=41.98183184208729，输出一圈2015.12792842019
        self.encoder_resolution = 2015.12792842019
        # 编码速度转换值
        self.speed_rate = 100
        if perimeter is None:
            perimeter = 0.06 * math.pi
        self.dis_resolution = perimeter / self.encoder_resolution

    def set_perimeter(self, perimeter):
        self.dis_resolution = perimeter / self.encoder_resolution
        # print(self.dis_resolution)

    def set_diameter(self, diameter):
        self.dis_resolution = diameter * math.pi / self.encoder_resolution

    def sp2virtual(self, speed: np.any):
        # 速度转为encoder输出
        speed_encoder = speed / self.dis_resolution

        # encoder转为控制器设置值
        speed_out = speed_encoder / self.speed_rate
        # print(speed_out)
        speed_out = np.clip(speed_out, -100, 100).astype(np.int8)
        # print(speed_out)
        return speed_out

    def dis2true(self, encoder_dis):
        dis_out = encoder_dis * self.dis_resolution
        return dis_out

    def sp2true(self, speed):
        # 控制器速度转为encoder输出
        speed_encoder = int(speed * self.speed_rate)
        speed_out = speed_encoder * self.dis_resolution
        return speed_out

    def encoder2dis(self, encoder_dis):
        dis_out = encoder_dis * self.dis_resolution
        return dis_out

    def dis2encoder(self, dis):
        encoder_out = dis / self.dis_resolution
        return encoder_out


class NoneDev:
    def __init__(self) -> None:
        pass

    def not_support(self):
        logger.info("dev not support")
        while True:
            time.sleep(1)

    def read(self, *args, **kwargs):
        self.not_support()

    def get_stick(self, *args, **kwargs):
        self.not_support()

    def set_light(self, *args, **kwargs):
        self.not_support()

    def set_angle(self, *args, **kwargs):
        self.not_support()

    def show(self, *args, **kwargs):
        self.not_support()

    def reset(self, *args, **kwargs):
        self.not_support()

    def set_speed(self, *args, **kwargs):
        self.not_support()

    def rings(self, *args, **kwargs):
        self.not_support()

    def set_number(self, *args, **kwargs):
        self.not_support()

    def set_speed(self, *args, **kwargs):
        self.not_support()

    def get_encoders(self, *args, **kwargs):
        self.not_support()

    def reset_encoders(self, *args, **kwargs):
        self.not_support()


class Beep:
    def __init__(self) -> None:
        self.beep1 = Buzzer_1()
        self.beep2 = Buzzer_2()

    def rings(self, freq=200, duration=0.2):
        funcs = [self.beep1.rings, self.beep2.rings]
        funcs[ctl_id](freq, duration)
        # time.sleep(duration+0.3)


class AnalogInput:
    def __init__(self, port_id=None) -> None:
        self.sensor_1 = AnalogInput_1(port_id)
        self.sensor_2 = AnalogInput_2(port_id)

    def read(self):
        funcs = [self.sensor_1.read, self.sensor_2.no_act]
        return float(funcs[ctl_id]())


# 红外传感器
class Infrared:
    def __init__(self, port_id=None) -> None:
        self.infrared_1 = Infrared_1(port_id)
        self.infrared_2 = Infrared_2(port_id)

    def read(self):
        funcs = [self.infrared_1.read, self.infrared_2.no_act]
        # 模拟量的结果转为浮点数单位 m
        return funcs[ctl_id]() / 1000


class NixieTube:
    def __init__(self, port_id=None) -> None:
        self.nixie_tube_1 = NixieTube_1(port_id)
        self.nixie_tube_2 = NixieTube_2(port_id)

    def set_number(self, number):
        funcs = [self.nixie_tube_1.set_number, self.nixie_tube_2.set_number]
        return funcs[ctl_id](number)


class BluetoothPad:
    def __init__(self) -> None:
        # 调用父对象初始化
        self.blue_pad_1 = NoneDev()
        self.blue_pad_2 = BluetoothPad_2()

    def read(self):
        """
        获取蓝牙手柄的值：
        - return: [ 左摇杆x, 左摇杆y, 右摇杆x, 右摇杆y, 按键值 ]
        - 按键值:
            - sum = 2^key[0] +2^key[1] +...+ 2^key[15]，
            - `key[n]` 为第n个按键值，按下为n，未按下为0
        ---
        ```
        .╭────╮                            ╭────╮.
        .| 10 |                            | 11 |.
        ╭╰════╯────────────────────────────╰════╯╮
        │  ╭────╮       WhalesBot        ╭────╮  │
        │  │ 12 │                        │ 13 │  │
        │  ╰────╯  ╭──╮            ╭──╮  ╰────╯  │
        │          │14│            │15│          │
        │  ╭───────╰══╯╮          ╭╰══╯───────╮  │
        │  │     0     │          │     4     │  │
        │  │ 1 < 8 > 3 │          │ 7 < 9 > 5 │  │
        │  │     2     │          │     6     │  │
        │  ╰───────────╯          ╰───────────╯  │
        ╰────────────────────────────────────────╯
        ```"""
        funcs = [self.blue_pad_1.get_stick, self.blue_pad_2.get_stick]
        return funcs[ctl_id]()


class BoardKey:
    def __init__(self) -> None:
        self.board_key_1 = NoneDev()
        self.board_key_2 = BoardKey_2()

    def read(self):
        funcs = [self.board_key_1.read(), self.board_key_2.no_act]
        return funcs[ctl_id]()


class LedLight:
    def __init__(self, port_id=None) -> None:
        self.led = [LedLight_1(port_id), LedLight_2(port_id)]

    def set_light(self, led_id, r, g, b):
        return self.led[ctl_id].set_light(led_id, r, g, b)


class Key4Btn:
    def __init__(self, port_id=None) -> None:
        self.key4btn_1 = ButtonAll_1(port_id)
        self.key4btn_2 = Key4Btn_2(port_id)
        # self.key4btn = [ButtonAll_1(port_id), Key4Btn_2(port_id)]

    def read(self):
        funcs = [self.key4btn_1.get_btn, self.key4btn_2.get_btn]
        return funcs[ctl_id]()

    def get_key(self):
        funcs = [self.key4btn_1.clicked, self.key4btn_2.get_btn]
        return funcs[ctl_id]()

    def get_key_async(self, callback=None, timeout=None):
        """异步读取当前按键号(发命令不等应答)。回包时回调按键号, 超时回调 None。
        供按键轮询线程等不希望阻塞总线的场景; MC601 无异步能力, 回落到同步。
        """
        if ctl_id == 1:
            self.key4btn_2.get_key_async(callback=callback, timeout=timeout)
        else:
            if callback is not None:
                callback(self.key4btn_1.clicked())


class Motor4:
    def __init__(self) -> None:
        self.motor4_1 = Motor4_1()
        self.motor4_2 = Motor4_2()
        # self.encoders_1 = encoder4_sim_ctl1
        self.encoders_2 = EncoderMotors4_2()
        self.encoders_2.reset()

    def set_speed(self, speeds):
        funcs = [self.motor4_1.set_speed, self.motor4_2.set_speed]
        return funcs[ctl_id](speeds)

    def get_encoder(self):
        funcs = [self.motor4_1.get_encoders, self.encoders_2.get]
        return funcs[ctl_id]()

    def reset(self):
        funcs = [self.motor4_1.reset, self.encoders_2.reset]
        return funcs[ctl_id]()


class EncoderMotor:
    def __init__(self, port_id) -> None:
        self.encoder_1 = NoneDev()
        self.encoder_2 = EncoderMotor_2(port_id)

    def get_encoder(self):
        funcs = [self.encoder_1.read, self.encoder_2.get]
        return funcs[ctl_id]()

    def reset(self):
        funcs = [self.encoder_1.reset, self.encoder_2.reset]
        return funcs[ctl_id]()


class Motor:
    # 编码器一圈12栅格编码值48 , 减速比(28/11)^4=41.98183184208729，输出一圈2015.12792842019
    motor_resolutions = {"motor_280": 48 * (28 / 11) ** 4, "motor_280_0": 48 * 46}

    def __init__(self, port_id, reverse=1, type="motor_280") -> None:
        self.motor_1 = Motor_1(port=port_id)
        self.motor_2 = Motor_2(port_id, reverse=reverse)
        self.encoder_2 = EncoderMotor_2(port_id, reverse=reverse)
        self.encoder_2.reset()

        encoder_resolution = self.motor_resolutions[type]
        encoder2sp = 100
        # 弧度到编码器的比例
        self.rad2encoder = encoder_resolution / math.pi / 2
        self.encoder2rad = 1 / self.rad2encoder
        # 弧度到电机的虚拟速度的比例
        self.rad2virtual = self.rad2encoder / encoder2sp
        self.virtual2rad = 1 / self.rad2virtual

    def set_sp(self, speed):
        funcs = [self.motor_1.rotate, self.motor_2.set_speed]
        # print(speed)
        speed = limit_val(speed, -100, 100)
        # print(speed)
        return funcs[ctl_id](speed)

    def set_angular(self, angular):
        # print()
        return self.set_sp(self.rad2virtual * angular)

    def get_encoder(self):
        funcs = [self.motor_1.encoder, self.encoder_2.get_encoder]
        return funcs[ctl_id]()

    def get_rad(self):
        return self.get_encoder() * self.encoder2rad

    def reset(self):
        funcs = [self.motor_1.reset, self.motor_2.reset]
        if ctl_id == 1:
            self.encoder_2.reset()
        return funcs[ctl_id]()


class Motors:
    # 编码器一圈12栅格编码值48 , 减速比(28/11)^4=41.98183184208729，输出一圈2015.12792842019
    motor_resolutions = {"motor_280": 48 * (28 / 11) ** 4, "motor_280_0": 48 * 46}

    def __init__(self, port_list=None, reverse=False, type="motor_280") -> None:
        # print(type)
        encoder_resolution = self.motor_resolutions[type]
        encoder2sp = 100
        # 弧度到编码器的比例
        self.rad2encoder = encoder_resolution / math.pi / 2
        self.encoder2rad = 1 / self.rad2encoder
        # 弧度到电机的虚拟速度的比例
        self.rad2virtual = self.rad2encoder / encoder2sp
        self.virtual2rad = 1 / self.rad2virtual

        self.motors_1 = NoneDev()
        self.motors_2 = Motors_2(port_list, reverse)
        self.motor4_2 = Motor4_2()  
        self.reverse = reverse

    def set_speed(self, speeds):
        if ctl_id == 1:
            if not self.reverse:
                speeds = [-i for i in speeds]
            return self.motor4_2.set_speed(list(speeds))
        return self.motors_1.set_speed(speeds)

    def set_angular(self, angular):
        # print(self.encoder_resolution)
        sp_virtual = np.array(angular) * self.rad2virtual
        sp_virtual = np.clip(sp_virtual, -100, 100).astype(np.int8)
        # print(sp_linear)
        return self.set_speed(sp_virtual)

    def get_encoder(self):
        funcs = [self.motors_1.get_encoders, self.motors_2.get_encoder]
        return funcs[ctl_id]()

    # 获取弧度值
    def get_rad(self):
        encoder_last = np.array(self.get_encoder())
        # print(encoder_last)
        return encoder_last * self.encoder2rad

    def get_encoder_async(self, callback=None, timeout=None):
        """异步读取 4 路编码器(合并一帧, 不等应答立即返回)。回包时回调编码值列表。"""
        if ctl_id == 1:
            self.motors_2.get_encoder_async(callback=callback, timeout=timeout)
        else:
            # MC601: 无异步能力, 回落到同步
            if callback is not None:
                callback(self.get_encoder())

    def reset(self):
        funcs = [self.motors_1.reset, self.motors_2.reset_encoder]
        return funcs[ctl_id]()


class WheelWrap:
    def __init__(
        self, port_list=None, raduis=0.03, motor_type="motor_280", reverse=False
    ) -> None:
        self.motors = Motors(port_list, reverse, motor_type)
        self.raduis = raduis
        self.linear2rad = 1 / self.raduis

    def set_linear(self, vel_linear):
        # 线速度转角速度
        angular = np.array(vel_linear) * self.linear2rad
        return self.motors.set_angular(angular)

    def set_angular(self, angular):
        return self.motors.set_angular(angular)

    def get_rad(self):
        return self.motors.get_rad()

    def get_linear(self):
        d_linear = self.motors.get_rad() * self.raduis
        return d_linear

    def get_linear_async(self, callback=None, timeout=None):
        """异步读取轮子线速度(弧度编码器值 * 半径), 不等应答立即返回。回包时回调线速度列表。"""

        def on_enc(encoders):
            if encoders is None:
                if callback is not None:
                    callback(None)
                return
            if callback is not None:
                callback(np.array(encoders) * self.motors.encoder2rad * self.raduis)

        self.motors.get_encoder_async(callback=on_enc, timeout=timeout)

    def reset(self):
        return self.motors.reset()


class ServoPwm:
    def __init__(self, port_id=None, mode=180) -> None:
        self.mode = mode
        self.servo_1 = ServoPwm_1(port_id)
        self.servo_2 = ServoPwm_2(port_id)

    def set_angle(self, angle, speed=100):
        funcs = [self.servo_1.set_angle, self.servo_2.set_angle]
        angle = int(angle / self.mode * 180 + 90)
        return funcs[ctl_id](angle, speed)

    def set_angle_async(self, angle, speed=100, callback=None):
        """异步设置 PWM 舵机角度(发命令不等应答), 供四轴并发使用。
        ctl_id=1(MC602) 走异步; ctl_id=0(MC601) 无异步能力, 回落到同步 set_angle。
        """
        angle = int(angle / self.mode * 180 + 90)
        if ctl_id == 1:
            return self.servo_2.set_angle_async(angle, speed, callback=callback)
        return self.servo_1.set_angle(angle, speed)


class ServoBus:
    def __init__(self, port_id=None) -> None:
        self.servo_bus_1 = ServoBus_1(port_id)
        self.servo_bus_2 = ServoBus_2(port_id)
        logger.info(f"总线电机初始化完成，ID:{port_id}")

    def set_angle(self, angle, speed=100):
        funcs = [self.servo_bus_1.set_angle, self.servo_bus_2.set_angle]
        return funcs[ctl_id](angle, speed)

    def set_angle_async(self, angle, speed=100, callback=None):
        """异步设置总线舵机角度(发命令不等应答), 供四轴并发使用。
        ctl_id=1(MC602) 走异步; ctl_id=0(MC601) 无异步能力, 回落到同步 set_angle。
        """
        if ctl_id == 1:
            return self.servo_bus_2.set_angle_async(angle, speed, callback=callback)
        return self.servo_bus_1.set_angle(angle, speed)

    def set_speed(self, speed):
        funcs = [self.servo_bus_1.set_speed, self.servo_bus_2.set_speed]
        return funcs[ctl_id](speed)


class PoutD:
    def __init__(self, port):
        self.pout_1 = PortOut_1(port)
        self.pout_2 = PoutD_2(port)

    def set(self, val):
        func = [self.pout_1.out, self.pout_2.set]
        return func[ctl_id](val)


class ScreenShow:
    def __init__(self) -> None:
        self.screen_1 = NoneDev()
        self.screen_2 = ScreenShow_2()
        self.screen = [NoneDev(), ScreenShow_2()]

    def show(self, args):
        return self.screen[ctl_id].show(args)


class Battry:
    def __init__(self) -> None:
        self.battry = [NoneDev(), Battry_2()]

    def read(self):
        return self.battry[ctl_id].read()


# logger.info("start time:{}".format(time.time()))


class MotorWrap:
    def __init__(
        self, id=1, reverse=1, type="motor_280", perimeter=0.06 * math.pi
    ) -> None:
        self.motor = Motor(id, reverse, type)
        self.motor.reset()
        # self.pid = PID(0.18, 0.01, 0.0018, setpoint=0.0, output_limits=[-100, 100])
        self.rad2dis = perimeter / math.pi / 2
        self.dis2rad = 1 / self.rad2dis
        self.count_flag = CountRecord(10)

    def set_linear(self, vel_linear):
        # 线速度转角速度
        angular = vel_linear * self.dis2rad
        # print(angular)
        return self.motor.set_angular(angular)

    def set_angular(self, angular):
        return self.motor.set_angular(angular)

    def get_rad(self):
        return self.motor.get_rad()

    def get_dis(self):
        # print(self.motor.get_rad())
        return self.motor.get_rad() * self.rad2dis

    def reset(self):
        return self.motor.reset()


class StepperWrap:
    def __init__(self, id, reverse=1, perimeter=0.008) -> None:
        self.reverse = reverse
        self.stepper = Stepper_2(id)

        # 系数
        # gradient = 9
        # 步进值1.8度 8细分, 2相位
        self.stepper2rad = math.pi / 180 * 1.8 / 16
        self.rad2pwm = 16 * 180 / 1.8 / math.pi

        # 计算半径，即弧度转弧长的系数
        self.rad2dis = perimeter / math.pi / 2
        self.dis2rad = 1 / self.rad2dis

    def get_rad(self):
        return self.stepper.get_step() * self.stepper2rad * self.reverse

    def set_rad(self, rad, time=0.5):
        pid = PID(5, 0, 0)
        pid.setpoint = rad
        if time < 0.1:
            time = 0.1
        rad_vel = abs(self.get_rad() - rad) / time
        pid.output_limits = (-rad_vel, rad_vel)
        cnt = 0
        while True:
            # print(pid(self.get_rad()))
            # print(self.get_rad()-rad)
            if abs(self.get_rad() - rad) < 0.1:
                cnt += 1
                if cnt > 10:
                    break
            else:
                cnt = 0
            self.set_angular(pid(self.get_rad()))
        self.set_angular(0)

    def set_angular(self, angular):
        # print(angular*self.rad2pwm)
        return self.stepper.set(int(angular * self.rad2pwm * self.reverse))

    def set_velocity(self, velocity):
        return self.set_angular(velocity * self.dis2rad)

    def get_dis(self):
        return self.get_rad() * self.rad2dis

    def reset(self):
        return self.stepper.reset()


if __name__ == "__main__":
    beep = Beep()
    beep.rings(200, 0.2)
    battry = Battry()
    # rad1 = step1.get_dis()
    # rad1 = step1.get_rad()
    # for i in range(10):
    #     step1.set_rad(math.pi/5*2*i)
    #     time.sleep(1)
    # step1.set_angular(math.pi/5*4)
    # time.sleep(0.5)
    # step1.set_angular(0)
    # step1.set_velocity(0)
    # print("dis", step1.get_rad()-rad1)

    # print("st: {.d} end:{.d}  dis{.d}".format(dis1,step1.stepper.get(),step1.stepper.get() - dis1))
    motor5_wrap = MotorWrap(6, reverse=-1, type="motor_280", perimeter=0.06 / 12 * 8)
    sp = -0.1
    motor5_wrap.set_linear(sp)
    for i in range(100):
        time.sleep(0.01)
        motor5_wrap.set_linear(sp)
    # time.sleep(5)
    motor5_wrap.set_linear(0)
    # print(motor5_wrap.get_dis())
    # print(motor5_wrap.get_encoder())

    # motor5
    # wheels = WheelWrap([1, 2, 3, 4], 0.03)
    # wheels.reset()
    # motor5 = Motor(5, type="motor_280_0")
    # motor5.set_sp(-20)
    # time.sleep(1)
    # motor5.set_sp(0)
    # print(motor5.get_encoder())
    # for i in range(10):
    # wheels.set_linear([0.1, 0.1, 0.1, 0.1])
    # wheels.set_angular([math.pi, math.pi, math.pi])
    # time.sleep(0.1)
    # wheels.set_linear([0, 0, 0, 0])
    # ret = wheels.get_rad()
    # ret = wheels.get_linear()
    # print(ret)
    """
    motors = Motors([1])
    motors.reset()
    print(time.time())
    for i in range(40):
        motors.set_angular([math.pi])
        time.sleep(0.092)
    print(time.time())
    motors.set_angular([0])
    print(motors.get_encoder())
    time_last = time.time()
    i = 0
    while True:
        now = time.time()
        res = battry.read()
        if now - time_last > 1:
            break
        i += 1
        # try:
        #     fps = 1/(now - time_last)
        # except:
        #     fps = 100
        # time_last = now
        # print("fps:",fps, "  battry:",  battry.read())
        # time.sleep(0.1)
    print("fps:", i)
    
    # motor.set_speed(20)

    # motor2 = MotorWrap(port_id=3, reverse=-1)
    # motor2.set_angle(100)
    # motor2.fit_and_plot(20)
    # for i in range(16):
    #     motor2.set_angle(22.5)
    #     time.sleep(1)
    #     motor2.set_angle(-22.5)
    #     time.sleep(1)
    # motor2.set_angle(360)
    # while True:
    #     motor2.fit_and_plot()
    time.sleep(0.1)
    """
