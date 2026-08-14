# -*- coding: utf-8 -*-
"""OCR 与文心分析(OcrErnieMixin)(从 perception.py 拆分而来)。"""
import base64
import difflib
import re
import time

import cv2

from smartcar.whalesbot.tools import CountRecord

class OcrErnieMixin:


    def animal_image_analysis(self):
        dets = self.get_detection_results()
        if len(dets) <= 0:
            print("未检测到任何目标，无法裁剪")
            return None, None
        cls_id, det_id, label, score, x_c, y_c, w, h = dets[0]
        image = self.side_image.copy()

        # 将归一化坐标转换为像素坐标
        img_h, img_w = image.shape[:2]
        x_c = int((x_c + 1) / 2 * img_w)
        y_c = int((y_c + 1) / 2 * img_h)
        w = int(w * img_w / 2)
        h = int(h * img_h / 2)
        x1 = int(x_c - w / 2)
        y1 = int(y_c - h / 2)
        x2 = int(x_c + w / 2)
        y2 = int(y_c + h / 2)

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
                    # x1, y1, w, h = det_bbox
                    # # print(img.shape)
                    # # print(x1, y1, w, h)
                    # x1 = img.shape[1] * (1+x1) / 2 - img.shape[1] * w / 4
                    # x2 = x1 + img.shape[1] * w / 2
                    # y1 = img.shape[0] * (1+y1) / 2 - img.shape[0] * h / 4
                    # y2 = y1 + img.shape[0] * h / 2
                    # x1 = 0 if x1 < 0 else int(x1)
                    # x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                    # y1 = 0 if y1 < 0 else int(y1)
                    # y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                    # # print(x1, x2, y1, y2)

                    # 将归一化坐标转换为像素坐标
                    x_c, y_c, w, h = det_bbox
                    w *= 1.2
                    h *= 1.2
                    img_h, img_w = img.shape[:2]
                    x_c = int((x_c + 1) / 2 * img_w)
                    y_c = int((y_c + 1) / 2 * img_h)
                    w = int(w * img_w / 2)
                    h = int(h * img_h / 2)
                    x1 = int(x_c - w / 2)
                    y1 = int(y_c - h / 2)
                    x2 = int(x_c + w / 2)
                    y2 = int(y_c + h / 2)

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
                        # OCR 已停用(仅 lane + task 模型)
                        if self.ocr_rec is None:
                            return None
                        # x1, y1, w, h = det_bbox

                        # # print(img.shape)
                        # # print(x1, y1, w, h)
                        # x1 = img.shape[1] * (1 + x1) / 2 - img.shape[1] * w / 4
                        # x2 = x1 + img.shape[1] * w / 2
                        # y1 = img.shape[0] * (1 + y1) / 2 - img.shape[0] * w / 4
                        # y2 = y1 + img.shape[0] * h / 2
                        # x1 = 0 if x1 < 0 else int(x1)
                        # x2 = img.shape[1] if x2 > img.shape[1] else int(x2)
                        # y1 = 0 if y1 < 0 else int(y1)
                        # y2 = img.shape[0] if y2 > img.shape[0] else int(y2)
                        # # print(x1, x2, y1, y2)
                        # img_txt = img[y1:y2, x1:x2]
                                            # 将归一化坐标转换为像素坐标
                        x_c, y_c, w, h = det_bbox
                        w *= 1.1
                        h *= 1.1
                        img_h, img_w = img.shape[:2]
                        x_c = int((x_c + 1) / 2 * img_w)
                        y_c = int((y_c + 1) / 2 * img_h)
                        w = int(w * img_w / 2)
                        h = int(h * img_h / 2)
                        x1 = int(x_c - w / 2)
                        y1 = int(y_c - h / 2)
                        x2 = int(x_c + w / 2)
                        y2 = int(y_c + h / 2)

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


    def yiyan_get_humattr(self, text):
        """
        获取人类属性分析

        使用文心一言分析文本中的人类属性信息。

        参数:
            text: 包含人类属性信息的文本

        返回:
            dict: 人类属性分析结果
        """
        return self.hum_analysis.get_res_json(text)


    def yiyan_get_actions(self, text):
        """
        获取动作分析

        使用文心一言分析文本中的动作信息。

        参数:
            text: 包含动作信息的文本

        返回:
            dict: 动作分析结果
        """
        return self.action_bot.get_res_json(text)
