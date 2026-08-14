# -*- coding: utf-8 -*-
"""推理与文心接口初始化(InferInitMixin)(从 perception.py 拆分而来)。"""
from smartcar.paddlebaidu.ernie_bot import ErnieBotWrap, OrderPrompt
from smartcar.paddlebaidu.infer_cs import ClintInterface


class InferInitMixin:

    def paddle_infer_init(self):
        """
        初始化Paddle推理

        初始化车道保持、前置方向识别、任务识别和OCR识别的推理接口。
        """
        # 前置巡线
        self.crusie = ClintInterface("lane")
        # 前置左右方向识别
        # self.front_det = ClintInterface('front')
        # 任务识别
        self.task_det = ClintInterface("task")
        # ocr识别(已停用:仅保留 lane + task 两个模型,无需 OCR)
        # self.ocr_rec = ClintInterface("ocr")
        self.ocr_rec = None
        # 识别为None
        self.last_det = None

    def ernie_bot_init(self):
        """
        初始化文心一言分析

        初始化、图像分析和订单分析的文心一言接口。
        """
        self.image_analysis = ErnieBotWrap()

        self.order_analysis = ErnieBotWrap()
        self.order_analysis.set_promt(str(OrderPrompt()))
