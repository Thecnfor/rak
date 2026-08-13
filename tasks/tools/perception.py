# -*- coding: utf-8 -*-
"""推理 / 检测 / OCR / 文心一言相关方法（从 car.py 拆分而来）。

这些方法组合进 MyCar(MotionMixin, PerceptionMixin, MecanumDriver) 使用，
通过 self 访问摄像头、推理客户端与机械臂等硬件接口。
"""
import base64
import difflib
import json
import re
import threading
import time

import cv2
import zmq
from typing import List

from smartcar.paddlebaidu.ernie_bot import ErnieBotWrap, OrderPrompt
from smartcar.paddlebaidu.infer_cs import ClintInterface
from smartcar.whalesbot.tools import CountRecord, logger

# 侧视检测结果的叠框持续时间(秒): 此窗口内经检测后就显示带框画面
SIDE_OVERLAY_HOLD_SECONDS = 2.0


class PerceptionMixin:

    # 侧视实时流: 始终推画面, 检测线程每 0.5s 跑一次检测, 有目标就叠框
    _side_stream_flag = False

    def start_side_stream(self):
        """启动侧视(cam2)实时流 + 持续检测线程, 由 MyCar 初始化时调用。"""
        if PerceptionMixin._side_stream_flag:
            return
        PerceptionMixin._side_stream_flag = True
        self._init_realtime_cache()
        thread = threading.Thread(
            target=self._side_stream_loop, name="side_stream", daemon=True)
        thread.start()

    def _init_realtime_cache(self):
        """初始化实时检测结果缓存(后台线程每 0.5s 更新一次)。"""
        self._det_lock = threading.Lock()
        self._det_cache = None  # (timestamp, dets, annotated_frame, raw_frame)

    def get_realtime_detections(self, fresh=False, max_age=None):
        """实时获取侧视 task 检测结果。

        后台线程持续检测(_side_stream_loop 每 0.5s 更新缓存), 本方法非阻塞返回
        最新一次结果; fresh=True 时立刻同步跑一次推理(独立连接, 不阻塞后台线程)。

        返回:
            list: [cls_id, obj_id, label, score, x_c, y_c, w, h](归一化)
        """
        if fresh:
            try:
                raw = self.cap_side.read()
            except Exception:
                return []
            if raw is None:
                return []
            sock = self._side_detect_client()
            try:
                dets = self._side_detect(sock, raw)
                annotated = self.draw_detection_results(raw.copy(), dets)
                with self._det_lock:
                    self._det_cache = (time.time(), dets, annotated, raw)
            except Exception as e:
                logger.warning(f"实时检测(fresh)失败: {e}")
                return []
            finally:
                try:
                    sock.close(linger=0)
                except Exception:
                    pass
            return dets

        cache = self._get_det_cache()
        if cache is None:
            return []
        ts, dets, _, _ = cache
        if max_age is not None and time.time() - ts > max_age:
            return []
        return dets

    def get_realtime_side_frame(self, with_overlay=True):
        """获取后台线程最近一次侧视画面(带框/原图), 无缓存时返回 None。"""
        cache = self._get_det_cache()
        if cache is None:
            return None
        _, _, annotated, raw = cache
        return annotated if (with_overlay and annotated is not None) else raw

    def _get_det_cache(self):
        lock = getattr(self, "_det_lock", None)
        if lock is None:
            self._init_realtime_cache()
            lock = self._det_lock
        with lock:
            return self._det_cache

    def _side_detect_client(self):
        """创建独立于任务检测的 ZMQ 客户端(避免共享 socket 的线程竞争)。"""
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.RCVTIMEO = 5000
        sock.connect("tcp://127.0.0.1:5002")
        try:
            sock.send(b"ATATA")
            sock.recv()
        except Exception:
            pass
        return sock

    def _side_detect(self, sock, raw):
        """在侧视图上跑一次检测, 返回 [cls,obj,label,score, nx,ny,nw,nh] 列表。"""
        ok, buf = cv2.imencode(".jpg", raw)
        if not ok:
            return []
        sock.send(b"image" + buf.tobytes())
        res = json.loads(sock.recv())
        return res if isinstance(res, list) else []

    def _side_stream_loop(self):
        # 后台线程只负责推画面: 检测结果新鲜时叠框, 否则推原图。
        # 检测完全由 get_realtime_detections(fresh=True) / get_detection_results 驱动
        while not getattr(self, "_stop_flag", False):
            try:
                raw = self.cap_side.read()
                show = raw
                cache = self._get_det_cache()
                if cache is not None:
                    ts, _dets, annotated, _raw = cache
                    if (annotated is not None
                            and time.time() - ts < SIDE_OVERLAY_HOLD_SECONDS):
                        show = annotated
                if show is not None:
                    self.streamer.update_frame(show, "cam2")
            except Exception as e:
                logger.warning(f"侧视流转发异常: {e}")
            time.sleep(0.05)  # ~20fps
        PerceptionMixin._side_stream_flag = False

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
                            # if matcher > 0.85:
                            #     text_count(T)
                        # print(text)
                        # print(res.bbox)
                        # print(text)
                        # if text_count(text):
                        #     return text


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
        # 同步刷新实时检测缓存: 侧视流转发线程据此叠框,
        # get_realtime_detections() 也可直接读到本次结果
        self._get_det_cache()
        with self._det_lock:
            self._det_cache = (time.time(), det_task, image, self.side_image)
        # print(det_task)
        return det_task


    def get_lane_results(self):
        image = self.cap_front.read().copy()
        res = self.crusie(image)
        error, angle = res[0], res[1]
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
