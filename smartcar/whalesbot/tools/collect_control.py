#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Created:   2026-04-05
# remote_control.py - 双摄像头遥控车数据收集系统
"""
功能说明：
- 通过蓝牙手柄控制遥控车运动和机械臂操作
- 同时使用两个摄像头进行数据收集：cam1 车道数据，cam2 物体数据
- 独立的数据收集器类管理每个摄像头的数据保存和状态
- 双路流媒体推送到 Streamer 进行远程监控
- 按键控制数据收集、删除数据、清空数据
- 安全退出程序时自动停止车辆、保存数据、关闭资源

"""

import cv2
import threading
import time
import json
import subprocess
import os, sys


from ..vehicle import MecanumDriver, BluetoothPad, ScreenShow, Beep, ArmController, ServoPwm
from .camera import Camera
from .streamer import Streamer
from .log_wrap import logger


class DataCollector:
    """
    通用数据收集器类
    可实例化多个对象，分别管理不同摄像头的数据收集
    """
    def __init__(self, camera: Camera, save_dir: str, cam_id: str = "cam", loop_delay: float = 0.05):
        """
        初始化数据收集器
        :param camera: 摄像头实例
        :param save_dir: 数据保存目录
        :param cam_id: 摄像头标识（用于日志）
        """
        self.cap = camera
        self.cam_id = cam_id
        self.loop_delay = loop_delay
        # 配置保存目录
        # path_dir = os.path.abspath(os.path.dirname(__file__))  # 获取当前文件所在目录
        path_dir = os.getcwd() # 当前工作目录（项目根目录）
        self.save_dir = os.path.join(path_dir, save_dir)
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir, exist_ok=True)
            
        # 数据状态
        self.index = 0
        self.json_data = []
        self.json_path = os.path.join(self.save_dir, "data.json")
        
        # 控制标志
        self.run_flag = False       # 收集运行标志
        self.exit_flag = False      # 退出标志
        self.car_state = [0.0, 0.0, 0.0]  # 当前车辆状态
        
        # 启动收集线程
        self.collect_thread = threading.Thread(target=self._collect_process, args=())
        self.collect_thread.daemon = True
        self.collect_thread.start()
        
        logger.info(f"[{self.cam_id}] dataset init,dir: {self.save_dir}")

    def set_car_state(self, car_state):
        """设置当前车辆状态（用于保存到JSON）"""
        self.car_state = car_state.copy()

    def start(self):
        """开始收集数据"""
        if not self.run_flag:
            self.run_flag = True
            logger.info(f"[{self.cam_id}] dataset start!!")

    def stop(self):
        """停止收集数据"""
        if self.run_flag:
            self.run_flag = False
            self.save_json()
            logger.info(f"[{self.cam_id}] stoped,image_nums: {self.index}")

    def delete_last_n(self, n: int = 30):
        """删除最近 n 张图片和对应数据"""
        deleted_count = 0
        for i in range(n):
            try:
                data = self.json_data.pop()
                img_path = os.path.join(self.save_dir, data['img_path'])
                if os.path.exists(img_path):
                    os.remove(img_path)
                self.index -= 1
                deleted_count += 1
            except IndexError:
                break
        time.sleep(2)
        
        if deleted_count > 0:
            self.save_json()
            logger.info(f"[{self.cam_id}] 删除了最近 {deleted_count} 张图片")
        else:
            logger.info(f"[{self.cam_id}] 没有可删除的图片")
        
        return deleted_count

    def clear_all(self):
        """清空所有图片和数据"""
        # 删除所有 jpg 文件
        try:
            subprocess.run(["find", self.save_dir, "-name", "*.jpg", "-delete"], check=True)
        except:
            pass
            
        # 重置状态
        self.json_data = []
        self.index = 0
        
        # 删除 JSON 文件
        if os.path.exists(self.json_path):
            os.remove(self.json_path)
            
        logger.info(f"[{self.cam_id}] 已清空所有数据")
        time.sleep(2)

    def save_json(self):
        """保存 JSON 数据到文件"""
        try:
            with open(self.json_path, 'w') as fp:
                json.dump(self.json_data, fp)
        except Exception as e:
            logger.error(f"[{self.cam_id}] JSON 保存失败: {e}")

    def _collect_process(self):
        """数据收集主循环（线程内部运行）"""
        name_length = 4
        
        while not self.exit_flag:
            if self.run_flag:
                try:
                    # 1. 获取图片
                    image = self.cap.read()
                    if image is None:
                        time.sleep(0.05)
                        continue
                    
                    # 2. 生成文件名
                    img_name = (name_length - len(str(self.index))) * '0' + str(self.index) + '.jpg'
                    img_path = os.path.join(self.save_dir, img_name)
                    
                    # 3. 保存图片
                    cv2.imwrite(img_path, image)
                    
                    # 4. 保存数据
                    data_dict = {
                        "img_path": img_name,
                        "state": self.car_state.copy()
                    }
                    self.json_data.append(data_dict)
                    
                    # 5. 日志和定期保存
                    if self.index % 10 == 0:
                        logger.info(f"[{self.cam_id}] 保存图片: {img_name}")
                        self.save_json()
                    
                    self.index += 1
                    
                except Exception as e:
                    logger.error(f"[{self.cam_id}] 收集过程出错: {e}")
            
            time.sleep(self.loop_delay)

    def close(self):
        """关闭收集器"""
        self.exit_flag = True
        self.stop()
        if self.collect_thread.is_alive():
            self.collect_thread.join(timeout=1.0)
        logger.info(f"[{self.cam_id}] 数据收集器已关闭")


class CollectControlCar:
    def __init__(
            self, 
            cap1: Camera,
            cap2: Camera ,
            dir1 = "./dataset/image_set_lane", 
            dir2 = "./dataset/image_set_object"
        ) -> None:
        # ==================== 1. 双摄像头初始化 ====================
        # cam1: 原有车道摄像头（保持原有逻辑）
        if cap1 is None:
            self.cap1 = Camera(1, 320, 240)  # 保持原有参数
        else:
            self.cap1 = cap1
            
        # cam2: 新增摄像头
        if cap2 is None:
            self.cap2 = Camera(2, 640, 480)  # 可根据需要调整参数
        else:
            self.cap2 = cap2

        # ==================== 2. 遥控车基础组件 ====================
        self.car = MecanumDriver()
        self.arm = ArmController()
        self.rings = Beep()
        self.display = ScreenShow()
        self.blue_pad = BluetoothPad()
        

        # ==================== 3. 双路流媒体 ====================
        self.streamer = Streamer()
        
        # ==================== 4. 双路数据收集器 ====================
        # collector1: cam1 车道数据收集（原有逻辑，保存到 lane_imageset）
        self.collector1 = DataCollector(
            camera=self.cap1,
            save_dir=dir1,
            cam_id="cam1"
        )
        
        # collector2: cam2 数据收集（新增，保存到 object_imageset）
        self.collector2 = DataCollector(
            camera=self.cap2,
            save_dir=dir2,
            cam_id="cam2",
            loop_delay=0.2  # cam2 可以适当降低收集频率，减少资源占用
        )

        # ==================== 5. 车辆控制参数 ====================
        self.state_base = [0.15, 0.15, 0.3]
        self.state_start = [0.3, 0.3, 0.5]
        self.car_state = [0.0, 0.0, 0.0]
        
        # ==================== 6. 控制标志 ====================
        self.exit_flag = False
        self.cam2_collect_toggle = False  # cam2 收集切换状态

        # ==================== 7. 启动系统 ====================
        self.beep()
        logger.info("双摄像头遥控车系统启动!!")
        
        # 启动流媒体线程
        self.stream_thread = threading.Thread(target=self._stream_process, args=())
        self.stream_thread.daemon = True
        self.stream_thread.start()
        

        
        # 进入车主循环
        self.car_process()

    def beep(self):
        self.rings.rings()



    def _stream_process(self):
        """双路流媒体推送循环"""
        while not self.exit_flag:
            try:
                # 读取两个摄像头的画面
                frame1 = self.cap1.read()
                frame2 = self.cap2.read()
                
                # 推送到 Streamer
                if frame1 is not None:
                    self.streamer.update_frame(frame1, 'cam1')
                if frame2 is not None:
                    self.streamer.update_frame(frame2, 'cam2')

            except Exception as e:
                logger.error(f"流媒体推送出错: {e}")
            
            time.sleep(0.033)  # 约 30 FPS

    def car_process(self):
        """遥控车主循环（处理按键、车辆控制）"""

        grasp_flag = False
        self.arm.grasp(grasp_flag)
        servo_1_angle_list = [-42,165]
        servo_1_flag = 0
        servo_1 = ServoPwm(1,180)
        servo_1.set_angle(servo_1_angle_list[servo_1_flag])

        while not self.exit_flag:
            keys_val = self.blue_pad.read()
            
            # ==================== 1. 蓝牙手柄连接检测 ====================
            if keys_val == [-1, -1, -1, -1, 0]:
                self.car_state = [0.0, 0.0, 0.0]
                logger.error("未检测到蓝牙手柄")
                self.display.show("can't find bluetooth pad\n")
                self.beep()
                time.sleep(1)
                continue

            # ==================== 2. 更新数据收集器的车辆状态 ====================
            self.collector1.set_car_state(self.car_state)
            self.collector2.set_car_state(self.car_state)

            # ==================== 3. 按键处理 ====================
            # --- cam1 原有按键逻辑（完全保持不变）---
            if keys_val[4] == 1<<10 :  # 按键 3: cam1 开始记录
                self.collector1.start()
            else:
                self.collector1.stop()
                
            if keys_val[4] == (1<<14)|(1<<15):  # 同时按下1和2退出程序
                self.close()
                break
                
            elif keys_val[4] == (1<<14)|(1<<2):  # 按键 1+V: 删除 cam1 最近 30 张
                self.collector1.delete_last_n(30)
                self.beep()
                self.display.show(f"cam1: {self.collector1.index}\n")
                
            elif keys_val[4] == (1<<14)|(1<<8):  # 按键 1+O: 清空 cam1 所有数据
                self.collector1.clear_all()
                self.beep()
                self.beep()
                self.display.show(f"cam1: {self.collector1.index}\n")

            # --- cam2 新增按键逻辑 ---
            if keys_val[4] == (1 << 11):  # 按键 4: cam2 开始记录
                self.collector2.start()
            else:
                self.collector2.stop()
                    
            if keys_val[4] == (1 << 15) | (1 << 6):  # 按键 2+∇: 删除 cam2 最近 30 张
                self.collector2.delete_last_n(30)
                self.beep()
                
            elif keys_val[4] == (1 << 15) | (1 << 9):  # 按键 2+▢: 清空 cam2 所有数据
                self.collector2.clear_all()
                self.beep()
                self.beep()

            # ==================== 4. 车辆运动控制（保持原有逻辑） ====================
            if self.collector1.run_flag:  # cam1 记录时使用 base 速度
                self.car_state[0] = self.state_base[0]
                self.car_state[1] = -1 * self.state_base[1] * keys_val[0]
                self.car_state[2] = -3.14 * self.state_base[2] * keys_val[2]
            else:  # 普通模式使用 start 速度
                self.car_state[0] = self.state_start[0] * keys_val[1]
                self.car_state[1] = -1 * self.state_start[1] * keys_val[0]
                self.car_state[2] = -3.14 * self.state_start[2] * keys_val[2]

            # 执行车辆控制
            self.car.set_velocity(*self.car_state)

            # 执行机械臂控制
            if keys_val[4] ==  (1 << 4):  # 按键△ : 向上移动机械臂
                self.arm.motor_y.set_velocity(0.5)
            elif keys_val[4] == (1 << 6):  # 按键▽: 向下移动机械臂
                self.arm.motor_y.set_velocity(-0.5)
            else:
                self.arm.motor_y.set_velocity(0.0)

            if keys_val[4] ==  (1 << 7):  # 按键◁ : 向左移动机械臂
                self.arm.motor_x.set_angular(50)
            elif keys_val[4] == (1 << 5):  # 按键▷: 向右移动机械臂
                self.arm.motor_x.set_angular(-50)
            else:
                self.arm.motor_x.set_angular(0.0)    

            if keys_val[4] ==  (1 << 0):  # 按键^ : 控制手臂向上<>^v
                self.arm.set_hand_angle("UP")
            elif keys_val[4] == (1 << 2):  # 按键V: 控制手臂向下<>^v
                self.arm.set_hand_angle("DOWN")
            
            if keys_val[4] ==  (1 << 1):
                self.arm.set_arm_angle("LEFT")
            elif keys_val[4] == (1 << 3):
                self.arm.set_arm_angle("RIGHT")
            
            if keys_val[4] ==  (1 << 9):
                grasp_flag = not grasp_flag
                self.arm.grasp(grasp_flag)
                time.sleep(0.3)
            if keys_val[4] ==  (1 << 8):
                servo_1_flag = (servo_1_flag+1)%2
                angle = servo_1_angle_list[servo_1_flag]
                print(angle)
                servo_1.set_angle(angle)
                time.sleep(0.3)
            time.sleep(0.05)

    def close(self):
        """关闭系统"""
        logger.info("正在关闭系统...")
        
        # 1. 停止车辆
        self.car.set_velocity(0.0, 0.0, 0.0)
        
        # 2. 关闭数据收集器
        self.collector1.close()
        self.collector2.close()
        
        # 3. 关闭流媒体
        self.streamer.stop()
        
        # 4. 关闭摄像头
        try:
            self.cap1.close()
            self.cap2.close()
        except:
            pass
            
        # 5. 退出提示
        self.exit_flag = True
        self.display.show(f"cam1_num: {self.collector1.index}\ncam2_num: {self.collector2.index}\n")
        
        # 6. 蜂鸣器提示
        for i in range(3):
            self.beep()
            time.sleep(0.4)
            
        logger.info("系统已安全关闭")


if __name__ == "__main__":
    # 初始化双摄像头
    # cam1: index=1, 320x240（保持原有参数）
    # cam2: index=2, 320x240（可根据需要调整）
    cam1 = Camera(1, 320, 240)
    cam2 = Camera(2, 640, 480)
    
    # 启动遥控车系统
    remote_car = CollectControlCar(cap1=cam1, cap2=cam2)