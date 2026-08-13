# -*- coding: utf-8 -*-

# 数据收集脚本

"""
功能说明：
- 通过蓝牙手柄控制遥控车运动和机械臂操作
- 同时使用两个摄像头进行数据收集：cam1 车道数据，cam2 物体数据
- 独立的数据收集器类管理每个摄像头的数据保存和状态
- 双路流媒体推送到 Streamer 进行远程监控
- 按键控制数据收集、删除数据、清空数据
- 安全退出程序时自动停止车辆、保存数据、关闭资源

操作说明：
- 正常摇杆控制车辆运动
- 按下【3】键 开始巡线数据采集
- 按下【4】键 开始侧边摄像头数据采集
- 同时按下【1】【2】 退出

"""
from smartcar import logger
from smartcar import CollectControlCar,Camera

logger.info("log测试")
if __name__ == "__main__":
    # 初始化双摄像头
    # cam1: index=1, 320x240（保持原有参数）
    # cam2: index=2, 320x240（可根据需要调整）
    cam1 = Camera(1, 320, 240)
    cam2 = Camera(2, 640, 480)
    
    # 启动遥控车系统
    dir1 = "dataset/image_set_lane"
    dir2 = "dataset/image_set_object"
    remote_car = CollectControlCar(cap1=cam1, cap2=cam2, dir1=dir1, dir2=dir2)