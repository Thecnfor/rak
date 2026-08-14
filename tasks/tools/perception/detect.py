# -*- coding: utf-8 -*-
"""感知查询(DetectMixin): 目标检测、巡线结果、目标定位与结果可视化(从 perception.py 拆分而来)。"""
import time
from typing import List

import cv2

from smartcar import logger

class DetectMixin:

    # 巡线滤波系数(一阶低通 EMA, 0~1, 越大越跟随, 越小越平滑)
    _lane_ema = 0.35
    # 巡线结果缓存: 用于推理异常时的上一帧保持
    _lane_last = (0.0, 0.0)
    # 单次推理异常允许的最大时长(秒), 超时则按无误差直行处理
    _lane_timeout = 0.3


    def get_detection_results(
        self, sort_pos=(0, 0), limit_x=1, limit_y=1
    ) -> List[list]:
        """
        获取检测结果,使用任务的目标检测对侧边摄像头图像进行检测，返回检测结果。

        返回:
            list: - 检测结果列表，每个元素包含 [cls_id, det_id, label, score, x_c, y_c, w, h]
        """
        self.side_image = self.cap_side.read()
        image = self.side_image.copy()
        det_task = self.task_det(image)
        det_task = [det for det in det_task if abs(det[4]) <= limit_x]
        det_task = [det for det in det_task if abs(det[5]) <= limit_y]

        det_task.sort(
            key=lambda x: (x[4] - sort_pos[0]) ** 2 + (x[5] - sort_pos[1]) ** 2
        )  # 按照距离由近及远排序
        image = self.draw_detection_results(image, det_task)
        # 同步刷新实时检测缓存: 推流线程据此叠框,
        # get_realtime_detections() 也可直接读到本次结果
        self._get_det_cache()
        with self._det_lock:
            self._det_cache = (time.time(), det_task)
        # print(det_task)
        return det_task


    def get_lane_results(self):
        """获取滤波后的巡线结果。

        前置摄像头推理得到 (error, angle):
            error: 中线误差(道路正中间相对车头中线的横向偏差)
            angle: 转弯误差(车头相对车道方向的夹角)
        返回前对两路做一阶低通(EMA)滤波, 抑制单帧抖动;
        推理失败/超时时保持上一帧(最多 _lane_timeout 秒, 之后按 0 处理直行)。
        """
        ts = time.time()
        try:
            image = self.cap_front.read().copy()
            if image is None:
                raise ValueError("cap_front 无画面")
            res = self.crusie(image)
            if not isinstance(res, (list, tuple)) or len(res) < 2:
                raise ValueError(f"推理结果异常: {res}")
            error, angle = float(res[0]), float(res[1])
        except Exception as e:
            logger.warning(f"巡线推理失败({e}), 保持上一帧")
            # 丢帧/异常: 超时内保持上一帧, 超时后按无误差直行
            last_ts = getattr(self, "_lane_last_ts", 0.0)
            if time.time() - last_ts > self._lane_timeout:
                return 0.0, 0.0
            return self._lane_last

        # 一阶低通滤波, 平滑单帧噪声
        l_e, l_a = self._lane_last
        error = l_e + self._lane_ema * (error - l_e)
        angle = l_a + self._lane_ema * (angle - l_a)
        self._lane_last = (error, angle)
        self._lane_last_ts = ts

        # 绘制标签
        label_text = f"d_e: {error:7.5f} d_a:{angle:7.5f}"
        # 用统一厚度偏移描边(黑边+绿字), 避免 cv2 5.x 下
        # 不同 thickness 渲染字形宽度不一致导致的白绿两层错位
        org = (20, 40)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                cv2.putText(
                    image, label_text, (org[0] + dx, org[1] + dy),
                    cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(
            image, label_text, org,
            cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 1, cv2.LINE_AA)
        self.streamer.update_frame(image, "cam1")
        # print(label_text)
        return error, angle


    def get_target_location(self, det):
        """
        通过传入的目标在图像的坐标，计算目标相对小车的偏移 x,y

        参数:
            det: 包含目标检测信息的列表，格式为 [cls_id, obj_id,label, score, x_c, y_c, w, h]
                - x_c: 目标在图像中的 x 坐标
                - y_c: 目标在图像中的 y 坐标
                - w: 目标的宽度
                - h: 目标的高度

        返回:
            tuple: 目标相对小车的坐标 (loc_x, loc_y)
                - loc_x: 目标相对小车的 x 坐标
                - loc_y: 目标相对小车的 y 坐标
        """
        # 摄像头图像在现实中实际的高和宽
        CAMERA_HEIGHT = 0.23
        CAMERA_WIDTH = 0.33
        # 机械臂x原点距离小车中心的距离
        ARM_OFFSET = 0.15

        # 获取机械臂的方向和长度
        arm_y = self.arm.x_pose_now + ARM_OFFSET
        side = self.arm.side
        length = 0

        # 根据机械臂方向调整长度
        if side == "RIGHT":
            length = -self.arm.arm_length
        elif side == "LEFT":
            length = self.arm.arm_length

        # 提取目标在图像中的坐标和尺寸
        x_c, y_c, w, h = det[4:]

        # 计算目标中心点在摄像头中的世界坐标
        x = CAMERA_WIDTH * (x_c + w / 2)
        y = CAMERA_HEIGHT * (y_c + h / 2)

        # 计算目标中心点在小车中的世界坐标
        loc_x = x
        loc_y = y + arm_y + length

        return loc_x, loc_y


    def draw_detection_results(self, img, dets_ret):
        """
        将检测结果绘制在图像上

        Args:
            img: 原始图像
            dets_ret: 检测结果列表，每个元素包含 [cls_id, det_id, label, score, x_c, y_c, w, h]

        Returns:
            绘制了检测结果的图像
        """
        # 创建图像副本，避免修改原始图像
        img_show = img.copy()

        # 遍历每个检测结果
        for index, det in enumerate(dets_ret):
            # [cls_id:6 obj_id:0 label:water_l2 score:0.955 bbox:[309 334 399 431]]
            det_cls_id, det_id, det_label, det_score, det_bbox = (
                det[0],
                det[1],
                det[2],
                det[3],
                det[4:],
            )
            x_c, y_c, w, h = det_bbox

            # 将归一化坐标转换为像素坐标
            img_h, img_w = img.shape[:2]
            x_c = int((x_c + 1) / 2 * img_w)
            y_c = int((y_c + 1) / 2 * img_h)
            w = int(w * img_w / 2)
            h = int(h * img_h / 2)
            x1 = int(x_c - w / 2)
            y1 = int(y_c - h / 2)
            x2 = int(x_c + w / 2)
            y2 = int(y_c + h / 2)

            # 绘制矩形框
            cv2.rectangle(img_show, (x1, y1), (x2, y2), (0, 255, 0), 1)

            # 绘制标签
            label_text = f"{index}-{det_label}:{det_score:.2f}"
            cv2.putText(
                img_show,
                label_text,
                (x1, y1),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        return img_show
