#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
import struct
import sys, os

# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

# 导入自定义log模块
from ...tools.log_wrap import logger
from .serial_wrap import serial_wrap

# 全局变量
serial_mc601 = serial_wrap

# 机器省电关闭
def eco_mode_off():
    cmd_data = bytes.fromhex('77 68 03 00 02 67 0A')
    for i in range(0, 2):
        serial_mc601.write(cmd_data)
        time.sleep(0.2)

class Button_1:
    BUTTON = {"1": "01", "2": "02", "3": "03", "4": "04"}

    def __init__(self, port, button):
        self.state = False
        self.port = port
        button_str = self.BUTTON[button]
        port_str = '{:02x}'.format(port)
        self.cmd_data = bytes.fromhex('77 68 05 00 01 DB {} {} 0A'.format(port_str, button_str))

    def clicked(self):
        response = serial_mc601.get_answer1(self.cmd_data)
        # print(response)
        if len(response) == 8 and response[4] == 0xDB and response[5] == self.port:
            button_byte = response[3]
            # print("button_byte=%x"%button_byte)
            if button_byte == 0x01:
                return True
        return False


class ButtonAll_1:
    # btn_sta = [[False, 0.0, 0.0], [False, 0.0, 0.0], [False, 0.0, 0.0], [False, 0.0, 0.0], [False, 0.0, 0.0]]
    btn_sta = []
    btn_sta_last = []
    state = []
    limit = 0.5
    bak_time = 0.0
    last_time = 0.0
    long_time = 0.7

    def __init__(self, port):
        self.port = port
        for i in range(5):
            self.btn_sta.append([False, 0.0, 0.0])
            # self.btn_sta.append([False, 0.0, 0.0, 0.0])
            # self.btn_sta_last.append([0, 0, 0.0])
        port_str = '{:02x}'.format(port)
        self.cmd_data = bytes.fromhex('77 68 05 00 01 E1 {} 00 0A'.format(port_str))

    def clicked(self):
        response = serial_mc601.get_answer1(self.cmd_data)
        # print("resp=",len(response))
        button_v = 0
        if len(response) == 9 and response[5] == 0xE1 and response[6] == self.port:
            button_byte = response[3:5] + bytes.fromhex('00 00')
            button_value = struct.unpack('<i', struct.pack('4B', *button_byte))[0]
            # print("button_value=%x"%button_value)
            if 0x80 <= button_value <= 0x28f:
                button_v = 3
            elif 0x300 <= button_value <= 0x48f:
                button_v = 1
            elif 0x501 <= button_value <= 0x6ff:
                button_v = 2
            elif 0x78f <= button_value <= 0x9ff:
                button_v = 4
            else:
                button_v = 0
        return button_v

    def get_btn(self):
        self.event()
        if len(self.state) > 0:
            key_v, key_state = self.state[0][0], self.state[0][1]
            del self.state[0]
            return key_v + 1 + (key_state-1)*4
        else:
            return 0

    def event(self):
        self.bak_time = time.time()
        index = 0
        while index < len(self.state):
            if self.bak_time - self.state[index][2] > self.limit:
                del self.state[index]
                continue
            index = + 1
        button_num = self.clicked()
        if button_num != 0:
            # print(button_num)
            for i in range(4):
                if i == button_num - 1:
                    tmp_time = self.btn_sta[i][1]
                    self.btn_sta[i][0] = True
                    self.btn_sta[i][1] += (self.bak_time - self.last_time)
                    self.btn_sta[i][2] = self.bak_time
                    if tmp_time < self.long_time < self.btn_sta[i][1]:
                        self.state.append([i, 2, self.btn_sta[i][2]])
                elif button_num != 0:
                    self.btn_sta[i][0] = False
                    self.btn_sta[i][1] = 0.0
            self.btn_sta[4][0] = False
            self.btn_sta[4][1] = 0.0
        else:
            for i in range(4):
                if self.btn_sta[i][0]:

                    if self.btn_sta[i][1] < self.long_time:
                        # self.btn_sta_last.append([i, 1, self.btn_sta[i][2]])
                        self.state.append([i, 1, self.btn_sta[i][2]])
                    self.btn_sta[i][0] = False
                    self.btn_sta[i][1] = 0.0
            self.btn_sta[4][0] = True
            self.btn_sta[4][1] += (self.bak_time - self.last_time)
        self.last_time = self.bak_time
        time.sleep(0.02)


class LimitSwitch_1:
    def __init__(self, port):
        self.port = port
        port_str = '{:02x}'.format(port)
        # print (port_str)
        self.cmd_data = bytes.fromhex('77 68 04 00 01 DD {} 0A'.format(port_str))

    def clicked(self):
        # serial_mc601.write(self.cmd_data)
        response = serial_mc601.get_answer1(self.cmd_data)  # 77 68 01 00 0D 0A
        # print(response)
        if len(response) < 8 or response[4] != 0xDD or response[5] != self.port \
                or response[2] != 0x01:
            return False
        state = response[3] == 0x01
        # print("state=",state)
        # print("elf.state=", self.state)
        # clicked = False
        if state:
            clicked = True
        else:
            clicked = False
        # print('clicked=',clicked)
        return clicked


class UltrasonicSensor_1:
    def __init__(self, port):
        self.port = port
        port_str = '{:02x}'.format(port)
        self.cmd_data = bytes.fromhex('77 68 04 00 01 D1 {} 0A'.format(port_str))

    def read(self):
        return_data = serial_mc601.get_answer1(self.cmd_data)
        # print(return_data)
        if len(return_data) < 11 or return_data[7] != 0xD1 or return_data[8] != self.port:
            return 0
        # print(return_data.hex())
        return_data_ultrasonic = return_data[3:7]
        ultrasonic_sensor = struct.unpack('<f', struct.pack('4B', *return_data_ultrasonic))[0]
        # print(ultrasonic_sensor)
        return int(ultrasonic_sensor)


class ServoBus_1:
    def __init__(self, port):
        self.port_str = '{:02x}'.format(port)

    def set_angle(self, angle, speed):
        angle = int(angle)
        speed = int(speed)
        cmd_servo_data = (bytes.fromhex('77 68 08 00 02 36 {}'.format(self.port_str)) +
                          speed.to_bytes(1, byteorder='big', signed=True) +
                          angle.to_bytes(4, byteorder='little', signed=True) + bytes.fromhex('0A'))
        # print(cmd_servo_data.hex(' '))
        serial_mc601.reset_buffer()
        serial_mc601.write(cmd_servo_data)
        # serial_mc601.write(cmd_servo_data)
    
    def set_speed(self, speed):
        speed = int(speed)
        angle = 1
        cmd_servo_data = (bytes.fromhex('77 68 06 00 02 37 {}'.format(self.port_str)) +
                          speed.to_bytes(1, byteorder='big', signed=True) +
                          angle.to_bytes(1, byteorder='little', signed=True) +
                          bytes.fromhex('0A')
        )
        # print(cmd_servo_data.hex(' '))
        serial_mc601.reset_buffer()
        serial_mc601.write(cmd_servo_data)

    def reset(self):
        cmd_servo_data = bytes.fromhex('77 68 04 00 02 64 0A')
        serial_mc601.reset_buffer()
        serial_mc601.write(cmd_servo_data)

class ServoPwm_1:
    def __init__(self, port):
        self.port_str = '{:02x}'.format(port)

    def set_angle(self, angle, speed):
        angle = int(angle)
        speed = int(speed)
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
        if speed < 0:
            speed = 0
        elif speed > 180:
            speed = 180
        cmd_servo_data = (bytes.fromhex('77 68 06 00 02 0B') + bytes.fromhex(self.port_str) +
                          speed.to_bytes(1, byteorder='big', signed=False) +
                          angle.to_bytes(1, byteorder='big', signed=False) + bytes.fromhex('0A'))
        # speed.to_bytes(1, byteorder='big', signed=True)
        # tt.to_bytes(1, byteorder='big', signed=False) + bytes.fromhex('0A'))
        for i in range(5):
            serial_mc601.write(cmd_servo_data)
            time.sleep(0.005)


class LedLight_1:
    def __init__(self, port):
        self.port_str = '{:02x}'.format(port)

    def set_light(self, led_id, red, green, blue):  # 0代表全亮，其他值对应灯珠亮，1~4
        which_str = '{:02x}'.format(led_id)
        red_str = '{:02x}'.format(red)
        green_str = '{:02x}'.format(green)
        blue_str = '{:02x}'.format(blue)
        cmd_servo_data = bytes.fromhex('77 68 08 00 02 3B {} {} {} {} {} 0A'
                                       .format(self.port_str, which_str, red_str, green_str, blue_str))
        serial_mc601.write(cmd_servo_data)


class DigitOut_1:
    def __init__(self, port):
        self.port = port
        self.port_str = '{:02x}'.format(port)

    def out(self, value):  # 1断 2通
        value_str = '{:02x}'.format(value)
        cmd_servo_data = bytes.fromhex('77 68 05 00 02 1E {} {} 0A'.format(self.port_str, value_str))
        serial_mc601.write(cmd_servo_data)

from typing import List
import numpy as np

class EncoderMotorAllSim_1:
    def __init__(self):
        # 编码器一圈的值
        self.encoder_resolution = 2016
        # 编码器与速度转换值
        self.speed_rate = 100
        self.speed4 = np.array([[0, 0, 0, 0], [0, 0, 0, 0]])
        self.encoder4 = self.speed4.copy()
        self.last_time = time.time()

    # 计算速度变化时的编码器值
    def set_speed(self, speed4:List[int]):
        # 更新编码器的值
        self.encoder4 = self.encoder4 + (self.speed4 * self.speed_rate * (time.time() - self.last_time)).astype(np.int32)
        # 更新速度
        self.speed4 = np.array(speed4)
        self.last_time = time.time()

    # 速度不变时，获取编码器的值
    def get(self):
        # 根据最后一次更新的速度和时间，计算当前的编码器值
        encoder4 = self.encoder4 + (self.speed4 * self.speed_rate * (time.time() - self.last_time)).astype(np.int32)
        # print(self.encoder4)
        return encoder4

    def reset(self):
        self.encoder4 = np.array([0, 0, 0, 0])

# 定义所有模拟电机编码器的类
encoder_motor_all_sim1 = EncoderMotorAllSim_1()
# 77 68 06 00 01 e9 01 54 01 0A
class Motor_1:
    def __init__(self, driver_id=1, port=1):
        self.driver_id_str = '{:02x}'.format(driver_id)
        self.port_str = '{:02x}'.format(port)
        # 编码器与速度转换值
        self.speed_rate = 100
        self.speed = 0
        self.encoder = 0
        self.last_time = time.time()

    def rotate(self, speed):
        # print("---------------------")
        # 根据速度变化更新模拟编码值
        self.encoder += self.speed * self.speed_rate * (time.time() - self.last_time)
        self.last_time = time.time()
        # print("after", self.encoder)
        # print("---------------------")
        
        self.speed = speed
        cmd_servo_data = (bytes.fromhex('77 68 06 00 02 0C') + bytes.fromhex(self.driver_id_str) +
                          bytes.fromhex(self.port_str) + speed.to_bytes(1, byteorder='big', signed=True) +
                          bytes.fromhex('0A'))
        serial_mc601.write(cmd_servo_data)
    
    def get_encoder(self):
        # 根据最后一次更新的速度和时间，计算当前的模拟编码器值
        encoder = self.encoder + self.speed * self.speed_rate * (time.time() - self.last_time)
        return int(encoder)
    def reset_encoder(self):
        self.encoder = 0
        
    def reset(self):
        self.rotate(0)
        self.encoder = 0

# 电机编码器模拟，不是真的
class EncoderMotor4Sim_1:
    def __init__(self):
        # 编码器一圈的值
        self.encoder_resolution = 2016
        # 编码器与速度转换值
        self.speed_rate = 100
        self.speed4 = np.array([0, 0, 0, 0])
        self.speed4_last = np.array([0, 0, 0, 0])
        self.encoder4 = np.array([0, 0, 0, 0])
        self.last_time = time.time()

    # 计算速度变化时的编码器值
    def set_speed(self, speed4:List[int]):
        self.speed4_last = self.speed4
        self.speed4 = np.array(speed4)
        self.encoder4 = self.encoder4 + (self.speed4_last * self.speed_rate * (time.time() - self.last_time)).astype(np.int32)
        self.last_time = time.time()

    # 速度不变时，获取编码器的值
    def get(self):
        encoder4 = self.encoder4 + (self.speed4 * self.speed_rate * (time.time() - self.last_time)).astype(np.int32)
        # print(self.encoder4)
        return encoder4

    def reset(self):
        self.encoder4 = np.array([0, 0, 0, 0])
        
# encoder4_sim_ctl1 = EncoderMotor4Sim_1()
class Motor4_1:
    def __init__(self):
        self.comma_head_all_motor = bytes.fromhex('77 68 0c 00 02 7a 01')
        self.comma_trail = bytes.fromhex('0A')
        self.sp_struct = struct.Struct('>bbbb')
        self.encoder4_sim_ctl1 = EncoderMotor4Sim_1()

    def set_speed(self, speeds:List[int]):
        cmd_m4 = self.comma_head_all_motor
        for i in range(4):
            sp_bytes = (i+1).to_bytes(1, byteorder='big') + speeds[i].to_bytes(1, byteorder='big', signed=True)
            cmd_m4 += sp_bytes
        cmd_m4 += self.comma_trail
        # 模拟编码器给到速度
        self.encoder4_sim_ctl1.set_speed(speeds)
        # print(cmd_m4)
        serial_mc601.write(cmd_m4)

    def get_encoders(self):
        return self.encoder4_sim_ctl1.get()

    def reset(self):
        self.encoder4_sim_ctl1.reset()
        
class Infrared_1:
    def __init__(self, port):
        port_str = '{:02x}'.format(port)
        self.cmd_data = bytes.fromhex('77 68 04 00 01 D4 {} 0A'.format(port_str))

    def read(self):
        return_data = serial_mc601.get_answer1(self.cmd_data)
        # print(return_data)
        return_data_infrared = return_data[3:7]
        infrared_sensor = struct.unpack('<i', struct.pack('4B', *return_data_infrared))[0]
        return infrared_sensor


class Buzzer_1:
    def __init__(self):
        self.cmd_data = bytes.fromhex('77 68 05 00 02 3D 03 02 0A')

    def rings(self, *args):
        serial_mc601.write(self.cmd_data)
        # serial_mc601.get_answer
        time.sleep(0.4)
        # return_data = serial_mc601.read()
        # print("rings data:", return_data)


class MagneticSensor_1:
    def __init__(self, port):
        self.port = port
        port_str = '{:02x}'.format(self.port)
        self.cmd_data = bytes.fromhex('77 68 04 00 01 CF {} 0A'.format(port_str))

    def read(self):
        return_data = serial_mc601.get_anwser(self.cmd_data)
        # return_data = serial_mc601.read()
        # print("return_data=",return_data[8])
        if len(return_data) < 11 or return_data[7] != 0xCF or return_data[8] != self.port:
            return None
        # print(return_data.hex())
        return_data = return_data[3:7]
        mag_sensor = struct.unpack('<i', struct.pack('4B', *return_data))[0]
        return int(mag_sensor)


class AnalogInput_1:
    def __init__(self, port):
        self.port = port
        port_str = '{:02x}'.format(self.port)
        self.cmd_data = bytes.fromhex('77 68 04 00 01 E1 {} 0A'.format(port_str))
        self.last_val = 0

    def read(self):
        return_data = serial_mc601.get_anwser(self.cmd_data)
        # print("return_data=", return_data, "len:", len(return_data))
        if len(return_data) != 9 or return_data[-4] != 0xE1 or return_data[-3] != self.port:
            return self.last_val
        return_data = return_data[3:5]
        # print("return_data=",return_data)
        analog_sensor = struct.unpack('<h', return_data)[0]
        self.last_val = int(analog_sensor)
        return self.last_val

class PortOut_1:
    def __init__(self, port):
        self.port = port
        self.port_str = '{:02x}'.format(port)

    def out(self, value):  # 1断 2通
        value_str = '{:02x}'.format(value)
        cmd_servo_data = bytes.fromhex('77 68 05 00 02 3A {} {} 0A'.format(self.port_str, value_str))
        serial_mc601.write(cmd_servo_data)


class NixieTube_1:
    def __init__(self, port):
        # self.port = port
        self.port_str = '{:02x}'.format(port)

    def set_number(self, value):
        if value > 9999:
            value = 9999
        elif value < 0:
            value = 0
        value = int(value)
        value1 = value % 256
        value2 = int(value / 256)
        value1_str = '{:02x}'.format(value1)
        value2_str = '{:02x}'.format(value2)
        cmd_servo_data = bytes.fromhex('77 68 06 00 02 38 {} {} {} 0A'.format(self.port_str, value1_str, value2_str))
        serial_mc601.write(cmd_servo_data)

class AiCam_1:
    def __init__(self, port):
        self.port = port
        port_str = '{:02x}'.format(self.port)
        self.cmd_data = bytes.fromhex('77 68 06 00 01 E9 {} 54 18 0A'.format(port_str))

    def read(self):
        return_data = [0] * 8
        read_data = serial_mc601.get_answer1(self.cmd_data)
        time.sleep(0.03)
        # read_data = serial_mc601.read()
        # print("read_data=",read_data)
        if len(read_data) < 11 or read_data[-3] != self.port or read_data[-4] != 0xe9:
            return return_data
        # print(read_data.hex())
        # data = read_data[1:]
        data = read_data[3:(3 + 18)]
        # print(data)
        if data[0] != 0x63 or data[-1] != 0x0a:
            return return_data
        for i in range(8):
            return_data[i] = data[i * 2] + data[i * 2 + 1] * 255
        # print(return_data)
        return return_data
        # mag_sensor = struct.unpack('<i', struct.pack('4B', *(return_data)))[0]

def set_led():
    led_t = PortOut_1(2)
    ll = False
    if ll is False:
        led_t.out(1)
        ll = True
    else:
        led_t.out(2)
        ll = False


def button_update():
    button_all_test = ButtonAll_1(2)
    while True:
        button_num = button_all_test.clicked()
        print("button_value=", button_num)
        time.sleep(0.1)


def button_event():
    button_all = ButtonAll_1(2)
    while True:
        button_all.event()
        key_val = button_all.get_btn()
        if key_val[0] != 0:
            print(key_val)


def infrared_update():
    infrared_sensor = Infrared_1(1)
    while True:
        cg = infrared_sensor.read()
        print("------->>infrared_sensor=", cg)
        time.sleep(0.33)

def analog_update():
    analog_sensor = AnalogInput_1(1)
    while True:
        cg = analog_sensor.read()
        print("------->>analog_sensor=", cg)
        time.sleep(0.33)

def motor4_test():
    motor4 = Motor4_1()
    while True:
        motor4.set_speed([20, 20, 20, 20])
        time.sleep(1)
        motor4.set_speed([0, 0, 0, 0])
        time.sleep(1)
        motor4.set_speed([-20, -20, -20, -20])
        time.sleep(1)
        motor4.set_speed([0, 0, 0, 0])
        time.sleep(1)

def motor_test():
    motor1 = Motor_1(1)
    while True:
        motor1.rotate(20)
        # print(motor1.get_encoder())
        time.sleep(1)
        motor1.rotate(0)
        print(motor1.get_encoder())
        time.sleep(1)
        # print(motor1.get_encoder())

def servo_test():
    servo_bus = ServoBus_1(2)
    while True:
        servo_bus.set_angle(0, 100)
        time.sleep(1)
        servo_bus.set_angle(90, 100)
        time.sleep(1)

def servo_sp_test():
    servo_bus = ServoBus_1(2)
    while True:
        servo_bus.set_speed(0)
        time.sleep(1)
        servo_bus.set_speed(90)
        time.sleep(1)
        servo_bus.set_speed(0)
        time.sleep(1)
        servo_bus.set_speed(-90)
        time.sleep(1)

if __name__ == '__main__':
    serial_mc601.assert_dev("mc601")
    buzzer = Buzzer_1()
    buzzer.rings()
    motor_test()
    # analog_update()
    # servo_test()
    # servo_sp_test()
    # motor4_test()
    # nixie = NixieTube_1(3)
    # button_event()
    # infrared_update()
    # buzzer.rings()
    # nixie.display(0)
