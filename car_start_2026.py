#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
# 添加上本文件对应目录，用于导入car_wrap_2026和car_task_function
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from car_wrap_2026 import MyCar
from car_task_function import (
    init,
    auto_lane_tracing,
    auto_seeding,
    target_shooting_detection,
    water_tower_task,
    target_shooting,
    crop_harvesting,
    sort_and_store,
    get_order,
    order_delivery
)

def main():
    
    init()                                          # 初始化
    # auto_lane_tracing(speed=0.3, dis_hold=99)     # 巡线测试，99m可以保证巡线到基地
    auto_seeding()                                  # 播种任务
    animal_list = target_shooting_detection()       # 识别虫害
    water_tower_task()                              # 灌溉任务
    target_shooting(animal_list)                    # 射击除害
    crop_harvesting()                               # 作物收集
    sort_and_store()                                # 作物储存
    order_list = get_order()                        # 订单获取
    order_delivery(order_list)                      # 订单配送


if __name__ == "__main__":
    main()