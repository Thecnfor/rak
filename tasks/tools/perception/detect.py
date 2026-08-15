# -*- coding: utf-8 -*-
"""感知查询(DetectMixin): 目标检测、巡线结果、目标定位与结果可视化(从 perception.py 拆分而来)。"""
import time
from typing import List

import cv2

from smartcar import logger


class DetectMixin:

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

        前视 lane 推理由后台 _front_lane_loop 背靠背执行并写入实时缓存(含 EMA 滤波、
        异常保持、超时归零); 本方法非阻塞直接读缓存, 不做推理、不做绘制、不推流。
        """
        return self._get_lane_cache()

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
        CAMERA_WIDTH = 0.17

        # 获取摄像头图像的高度和宽度
        h, w = 480, 640
        # 计算摄像头单位像素的实际长度 x,y
        camera_dy = CAMERA_WIDTH / w
        camera_dx = CAMERA_HEIGHT / h
        # 计算目标在摄像头图像中的 x 坐标
        x_c, y_c = det[4], det[5]
        x_pixel = (x_c + 0.5) * w  # [-0.5, 0.5] -> [0, w]
        y_pixel = (y_c + 0.5) * h
        # 计算目标相对摄像头中心的像素偏移
        center_x = w / 2.0
        center_y = h / 2.0
        pixel_offset_x = x_pixel - center_x
        pixel_offset_y = y_pixel - center_y
        # 换算为真实距离偏移 (小车坐标: x 横向, y 纵向, 图像 y=远处 → loc_y 正方向)
        loc_x = -pixel_offset_x * camera_dy
        loc_y = -pixel_offset_y * camera_dx
        return loc_x, loc_y

    def draw_detection_results(self, img, dets_ret):
        """
        将检测结果绘制在图像上

        参数:
            img: 要绘制检测结果的图像
            dets_ret: 检测结果列表，每个元素包含 [cls_id, det_id, label, score, x_c, y_c, w, h]

        返回:
            绘制了检测结果的图像
        """
        # 获取图像高度和宽度
        h, w = img.shape[:2]
        # 遍历检测结果，绘制每个检测框和标签
        for det in dets_ret:
            # 解析检测结果
            cls_id, det_id, label, score, x_c, y_c, bw, bh = det[:8]
            # 计算检测框左上角坐标(x1, y1)和右下角坐标(x2, y2)
            x1 = int((x_c - bw / 2) * w)
            y1 = int((y_c - bh / 2) * h)
            x2 = int((x_c + bw / 2) * w)
            y2 = int((y_c + bh / 2) * h)

            # 根据类别 ID 选择不同颜色的检测框
            # 颜色列表，每个类别的颜色不同
            colors = [
                (0, 0, 255), (0, 255, 0), (255, 0, 0),
                (0, 255, 255), (255, 0, 255), (255, 255, 0),
                (128, 0, 128), (0, 128, 128), (128, 128, 0),
            ]
            # 根据类别 ID 选择颜色，取模防止越界
            color = colors[int(cls_id) % len(colors)]

            # 绘制检测框
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            # 绘制标签和置信度
            label_text = f"{label} {score:.2f}"
            # 在检测框上方绘制标签和置信度
            cv2.putText(
                img,
                label_text,
                (x1, max(30, y1 - 10)),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        # 返回绘制了检测结果的图像
        return img
