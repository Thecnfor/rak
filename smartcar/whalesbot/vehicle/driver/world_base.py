#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import math, threading
import numpy as np


from threading import Thread
import yaml, os, sys

import time
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 
# 导入自定义log模块
from log_info import logger
# 把该文件夹目录加入环境变量


def get_path_relative(*args):
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)


class WorldBase:
    def __init__(self):
        pass

    def speed_transform(self, vel_car, angle_car):
        '''
        速度转换
        '''
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # print(sin_car, cos_car)
        transform = np.array([[ cos_car, sin_car, 0],
                              [-sin_car, cos_car, 0],
                              [       0,       0, 1]])

        vel_world = np.array(vel_car).dot(transform)

        return vel_world
    
    def sp_world2car(self, vel_world, angle_car):
        '''
        世界坐标系到车坐标系
        '''
        sin_car = np.sin(angle_car)
        cos_car = np.cos(angle_car)
        # print(sin_car, cos_car)
        transform = np.array([[cos_car, -sin_car, 0], 
                              [sin_car,  cos_car, 0],
                              [       0,       0, 1]])
        vel_car = np.array(vel_world).dot(transform)

        return vel_car

class RoadMap():
    def __init__(self, path):
        self.path = path
        self.road_map = []
        self.road_map_index = 0
        self.road_map_length = 0
        self.load_road_map()
    
    def load_road_map(self):
        import json
        with open(self.path, "r", encoding='utf-8') as f:
            self.road_map = json.load(f)

if __name__ == "__main__":
    vel = [1, 0, 0]
    angle = 0
    print(WorldBase().speed_transform(vel, angle))