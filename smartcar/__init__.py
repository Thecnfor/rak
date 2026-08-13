"""Smartcar包的主模块

该模块提供了智能车相关的核心功能，包括：
- 摄像头控制
- 机械臂控制
- 车辆驱动
- 目标检测
- 自然语言处理
"""

# 从whalesbot.tools导入常用工具
from .whalesbot.tools import Camera, Streamer, logger, CountRecord, get_yaml, IndexWrap, PID, CollectControlCar
from .whalesbot.vehicle import (
    ArmController, ScreenShow, Key4Btn, Infrared, LedLight, MecanumDriver, Beep,
    Motors, Motor4, AnalogInput, Battry, BoardKey, NixieTube, ServoBus,
    ServoPwm, BluetoothPad, MotorConvert, WheelWrap, MotorWrap, PoutD, StepperWrap
)

# 从paddlebaidu导入常用组件
from .paddlebaidu.infer_cs import ClintInterface, Bbox
from .paddlebaidu.ernie_bot import ErnieBotWrap, HumAttrPrompt, ActionPrompt, ImagePrompt

# 导出常用组件供外部使用
__all__ = [
    # 摄像头和工具
    'Camera', 'Streamer', 'logger', 'CountRecord', 'get_yaml', 'IndexWrap', 'PID','CollectControlCar',
    # 车辆控制
    'ArmController', 'ScreenShow', 'Key4Btn', 'Infrared', 'LedLight', 'MecanumDriver', 'Beep',
    'Motors', 'Motor4', 'AnalogInput', 'Battry', 'BoardKey', 'NixieTube', 'ServoBus',
    'ServoPwm', 'BluetoothPad', 'MotorConvert', 'WheelWrap', 'MotorWrap', 'PoutD', 'StepperWrap',
    # 目标检测和NLP
    'ClintInterface', 'Bbox', 'ErnieBotWrap', 'HumAttrPrompt', 'ActionPrompt', 'ImagePrompt'
]