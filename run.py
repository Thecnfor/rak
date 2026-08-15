#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""完整比赛流程入口：按比赛顺序一行行调用任务函数。"""

from tasks import delivery, harvesting, ordering, seeding, shooting, sorting
from tasks import target_detection, watering
from tasks.tools import create_car


def main():
    car = create_car(reset=False)  # 初始化（含机械臂与里程计复位）
    try:
        while True:
            pass
        seeding.run(car)  # 播种任务
        # animal_list = target_detection.run(car)  # 识别虫害
        # watering.run(car)  # 灌溉任务
        # shooting.run(car, animal_list)  # 射击除害
        # harvesting.run(car)  # 作物收集
        # sorting.run(car)  # 作物储存
        # order_list = ordering.run(car)  # 订单获取
        # delivery.run(car, order_list)  # 订单配送
    finally:
        car.stop()
        car.close()


if __name__ == "__main__":
    main()
