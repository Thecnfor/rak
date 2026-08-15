# -*- coding: utf-8 -*-
"""OCR 与文心分析(OcrErnieMixin)(从 perception.py 拆分而来)。"""
import base64
import difflib
import re
import time

import cv2

from ..coords import norm_box_to_pixel
from smartcar.whalesbot.tools import CountRecord


class OcrErnieMixin:

    @staticmethod
    def _bbox_to_pixel(det_bbox, img_shape, scale=1.2):
        """归一化 bbox → 像素坐标 (带 padding)。"""
        x_c, y_c, w, h = det_bbox
        img_h, img_w = img_shape[:2]
        return norm_box_to_pixel(x_c, y_c, w, h, img_w, img_h, scale)

    def animal_image_analysis(self):
        dets = self.get_detection_results()
        if len(dets) <= 0:
            print("未检测到任何目标，无法裁剪")
            return None, None
        cls_id, det_id, label, score, x_c, y_c, w, h = dets[0]
        image = self.side_image.copy()

        # 将归一化坐标转换为像素坐标
        img_h, img_w = image.shape[:2]
        x1, y1, x2, y2 = norm_box_to_pixel(x_c, y_c, w, h, img_w, img_h)

        # img_h, img_w = image.shape[:2]

        # # 计算坐标 + 强制边界保护（核心修复！）
        # x1 = int(max(0, x_c - w / 2))
        # y1 = int(max(0, y_c - h / 2))
        # x2 = int(min(img_w, x_c + w / 2))
        # y2 = int(min(img_h, y_c + h / 2))
        # 防止裁剪出空图（核心修复！）
        if x2 <= x1 or y2 <= y1:
            print("裁剪区域无效，跳过")
            return None, None
        cropped_img = image[y1:y2, x1:x2]

        _, img_encoded = cv2.imencode(".jpg", cropped_img)
        # 转 base64 字符串
        base64_image = base64.b64encode(img_encoded.tobytes()).decode("utf-8")

        result, analysis = self.image_analysis.get_image_res(base64_image)
        print(f"image result: {result}  \nanalysis:{analysis}")
        return result, analysis

    def get_det_ocr(self, det, label="name", time_out=5.0):
        time_stop = time.time() + time_out
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        text_count = CountRecord(3)
        text_out = None
        print(det)
        while True:
            if self._stop_flag:
                return text_out
            if time.time() > time_stop:
                return text_out
            img = self.side_image
            if det is not None:
                det_cls_id, det_id, det_label, det_score, det_bbox = (
                    det[0],
                    det[1],
                    det[2],
                    det[3],
                    det[4:],
                )
                if label is not None:
                    flag = det_label == label
                else:
                    flag = det_label == "order" or det_label == "name"
                if flag:
                    # OCR 已停用(仅 lane + task 模型)
                    if self.ocr_rec is None:
                        return text_out
                    x1, y1, x2, y2 = self._bbox_to_pixel(det_bbox, img.shape, scale=1.2)

                    img_txt = img[y1:y2, x1:x2]

                    self.streamer.update_frame(img_txt, "cam1")
                    text = self.ocr_rec(img_txt)
                    print(f"当前检测文本: {text}")
                    text = "".join(re.findall(r"[\u4e00-\u9fffa-zA-Z]", text))
                    print(f"整理后文本: {text}")
                    if text_out is None:
                        text_out = text
                    else:
                        # 文本相似度比较
                        matcher = difflib.SequenceMatcher(None, text_out, text).ratio()
                        if text_count(matcher > 0.85):
                            return text_out
                        else:
                            text_out = text

    def get_ocr(self, label=None, time_out=3.0):
        """
        进行OCR识别

        使用侧面摄像头获取图像，进行文本检测和OCR识别，返回识别结果。

        参数:
            time_out: 超时时间（秒），默认为3

        返回:
            str: 识别到的文本，如果超时或未检测到则返回None
        """
        time_stop = time.time() + time_out
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        text_count = CountRecord(3)
        text_out = None
        while True:
            if self._stop_flag:
                return
            if time.time() > time_stop:
                return None
            dets = self.get_detection_results()

            img = self.side_image
            if len(dets) > 0:
                for det in dets:
                    det_cls_id, det_id, det_label, det_score, det_bbox = (
                        det[0],
                        det[1],
                        det[2],
                        det[3],
                        det[4:],
                    )
                    if label is not None:
                        flag = det_label == label
                    else:
                        flag = det_label == "order" or det_label == "name"
                    if flag:
                        if self.ocr_rec is None:
                            return None
                        x1, y1, x2, y2 = self._bbox_to_pixel(det_bbox, img.shape, scale=1.1)
                        img_txt = img[y1:y2, x1:x2]
                        self.streamer.update_frame(img_txt, "cam1")

                        text = self.ocr_rec(img_txt)
                        if text_out is None:
                            text_out = text
                        else:
                            # 文本相似度比较
                            matcher = difflib.SequenceMatcher(
                                None, text_out, text
                            ).ratio()
                            if text_count(matcher > 0.85):
                                return text_out
                            else:
                                text_out = text


