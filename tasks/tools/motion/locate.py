# -*- coding: utf-8 -*-
"""目标定位与机械臂逼近(LocateMixin): 检测目标对齐、世界坐标换算(从 motion.py 拆分而来)。"""
import math
import time

import cv2
from typing import Union

from ..coords import norm_box_to_pixel
from smartcar import PID, logger
from smartcar.whalesbot.tools import CountRecord, get_yaml
 

class LocateMixin:

    def lane_det_location(
        self,
        speed,
        pts_tar=[[0, 70, "text_det", 0, 0, 0, 0.70, 0.70]],
        dis_out=0.05,
        side=1,
        time_out=2,
        det="task",
    ):
        """
        侧面摄像头进行位置定位

        使用侧面摄像头检测目标并进行位置定位，通过PID控制调整车辆位置。

        参数:
            speed: 移动速度
            pts_tar: 目标点列表，每个元素包含 [id, 宽度, 标签, 置信度, x, y, w, h]
            dis_out: 距离限制，默认为0.05
            side: 方向，1为正方向，-1为反方向
            time_out: 超时时间（秒），默认为2
            det: 检测类型，默认为'task'

        返回:
            int: 目标索引，如果超时或距离超出限制则返回False
        """
        end_time = time.time() + time_out
        infer = self.task_det
        loc_pid = get_yaml(self.yaml_path)["location_pid"]  # type: ignore
        pid_x = PID(**loc_pid["pid_x"])
        pid_x.output_limits = (-speed, speed)
        pid_y = PID(**loc_pid["pid_y"])
        pid_y.output_limits = (-0.15, 0.15)
        # pid_w = PID(1.0, 0, 0.00, setpoint=0, output_limits=(-0.15, 0.15))

        # 用于相同记录结果的计数类
        x_count = CountRecord(5)
        dis_count = CountRecord(5)

        out_x = speed
        out_y = 0

        # 此时设置相对初始位置
        # self.set_pos_relative()
        # self.dis_tra_st = self.get_distance()
        x_st, y_st, _ = self.get_odometry()
        find_tar = False
        tar = []
        for pt_tar in pts_tar:
            # id, 物体宽度，置信度, 归一化bbox[x_c, y_c, w, h]
            tar_id, tar_width, tar_label, tar_score, tar_bbox = (
                pt_tar[0],
                pt_tar[1],
                pt_tar[2],
                pt_tar[3],
                pt_tar[4:],
            )
            tar_width *= 0.001
            tar_x, tar_y, tar_dis = self.det2pose(tar_bbox, tar_width)
            tar.append([tar_id, tar_width, tar_x, tar_y, tar_dis])
        # logger.info("tar x:{} dis:{}".format(tar_x, tar_dis))
        tar_id, tar_width, tar_x, tar_y, tar_dis = tar[0]
        pid_x.setpoint = tar_x
        pid_y.setpoint = tar_dis
        tar_index = 0
        flag_location = False
        while True:
            if self._stop_flag:
                return
            if time.time() > end_time:
                logger.info("time out")
                self.set_velocity(0, 0, 0)
                return False
            _pos_x, _pos_y, _pos_omage = self.get_odometry()  # 用来计算距离

            if abs(_pos_x - x_st) > dis_out or abs(_pos_y - y_st) > dis_out:
                if not find_tar:
                    logger.info("task location dis out")
                    self.set_velocity(0, 0, 0)
                    return False
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)

            img_side_show = img_side.copy()
            for det in dets_ret:
                det_cls_id, det_id, det_label, det_score, det_bbox = (
                    det[0],
                    det[1],
                    det[2],
                    det[3],
                    det[4:],
                )
                x_c, y_c, w, h = det_bbox
                # 将归一化坐标转换为像素坐标
                img_h, img_w = img_side.shape[:2]
                x1, y1, x2, y2 = norm_box_to_pixel(x_c, y_c, w, h, img_w, img_h)
                # 绘制矩形框
                cv2.rectangle(img_side_show, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # 绘制标签
                label_text = f"{det_label}:{det_score:.2f}"
                cv2.putText(
                    img_side_show,
                    label_text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            self.streamer.update_frame(img_side_show, "cam2")

            # dets_ret = self.mot_hum(img_side)
            # cv2.imshow("side", img_side)
            # cv2.waitKey(1)

            # 进行排序，此处排列按照自中心由近及远的顺序
            dets_ret.sort(key=lambda x: (x[4]) ** 2 + (x[5]) ** 2)
            print(dets_ret)

            # 如果没有，就重新获取
            if len(dets_ret) > 0:
                det = dets_ret[0]
                # 结果分解
                det_id, obj_id, det_label, det_score, det_bbox = (
                    det[0],
                    det[1],
                    det[2],
                    det[3],
                    det[4:],
                )
                # if find_tar is False:
                # tar_index = 0
                # for tar_pt in tar:
                for index, tar_pt in enumerate(tar):
                    if det_id == tar_pt[0]:
                        tar_index = index
                        tar_id, tar_width, tar_x, tar_y, tar_dis = tar_pt
                        pid_x.setpoint = tar_x
                        pid_y.setpoint = tar_dis
                        find_tar = True
                        # print("find tar", tar_id)
                        break

                if det_id == tar_id:
                    _x, _y, _dis = self.det2pose(det_bbox, tar_width)
                    out_x = pid_x(_x) * side  # type: ignore
                    out_y = pid_y(_dis) * side  # pyright: ignore[reportOptionalOperand]
                    # out_y = pid_y(_dis)
                    # out_y = pid_w(bbox_error[2])
                    # 检测偏差值连续小于阈值时，跳出循环
                    # print(bbox_error)
                    # print("err x:{:.2}, dis:{:.2}, tar x:{:.2}, tar dis:{:.2}".format(_x, _dis, tar_x, tar_dis))
                    flag_x = x_count(abs(_x - tar_x) < 0.01)
                    flag_dis = dis_count(abs(_dis - tar_dis) < 0.01)
                    if flag_x:
                        out_x = 0
                    if flag_dis:
                        out_y = 0
                    if flag_x and flag_dis:
                        logger.info("location{} ok".format(tar_id))
                        # flag_location = True
                        # 停止
                        self.set_velocity(0, 0, 0)
                        return tar_index

                # print("error_x:{:.2}, error_y:{:.2}, out_x:{:.2}, out_y:{:2}".format(bbox_error[0], bbox_error[2], out_x, out_y))
            else:
                x_count(False)
                dis_count(False)
            self.set_velocity(out_x, out_y, 0)

    def move_to_detection_target(
        self,
        delta_x=0.0,
        delta_y: Union[float, None] = 0.0,
        label=None,
        time_out=4.0,
        sort_pos=(0, 0),
        num=0,
    ):
        """
        前往目标位置

        参数:
            cls_id : 指定检测目标的 cls_id，默认None为距离中心最近的目标
            time_out: 设置超时时间
            包含目标检测信息的列表，格式为 [cls_id, obj_id,label, score, x_c, y_c, w, h]
        """
        time_stop = time.time() + time_out
        x_count = CountRecord(3)
        y_count = CountRecord(3)

        # pid_x.output_limits((-0.7, 0.7))

        out_x = 0
        out_y = 0
        # print(f"手柄方向：{self.arm.side}")
        if self.arm.side == "RIGHT":
            kp_y = -0.2
            kp_x = -0.25
            ki_x = 0.03
        else:
            kp_y = 0.2
            kp_x = 0.16
            ki_x = 0.0

        pid_x = PID(kp_x, ki_x)
        pid_x.output_limits = (-0.15, 0.15)
        pid_x.setpoint = delta_x
        while True:
            if self._stop_flag:
                self.set_velocity(0, 0, 0)
                self.arm.x_speed(0)
                return -1, "None"

            dets = self.get_detection_results(sort_pos=sort_pos)

            if label is not None:
                dets = [item for item in dets if item[2] == label]

            if len(dets) > num:
                det = dets[num]
                dx, dy = det[4:6]
                err_x = delta_x - dx
                if abs(err_x) < 0.015:
                    out_x = 0.0          # 足够近就不动, 让 CountRecord 确认稳定
                else:
                    out_x = -pid_x(dx)   # type: ignore

                if delta_y is None:
                    out_y = 0
                else:
                    out_y = kp_y * (dy - delta_y)

                flag_x = x_count(abs(dx) < 0.04)
                flag_y = y_count(abs(dy) < 0.02)
                if delta_y is None:
                    flag_y = True

                if flag_x:
                    out_x = 0
                if flag_y:
                    out_y = 0
                if flag_x and flag_y:
                    # logger.info(f"location{self.get_odometry()} ok, arm_pose{self.arm.x_pose_now}")
                    self.set_velocity(0, 0, 0)
                    self.arm.x_speed(0)
                    return det[0], det[2]
            else:
                x_count(False)
                y_count(False)
            self.set_velocity(out_x, 0, 0)
            self.arm.x_speed(out_y)
            time.sleep(0.05)

            if time.time() > time_stop:
                self.set_velocity(0, 0, 0)
                self.arm.x_speed(0)
                logger.error("对齐目标超时")
                # logger.info(f"location{self.get_odometry()} ok, arm_pose{self.arm.x_pose_now}")

                try:
                    return det[0], det[2]
                except:
                    return (None, None)

    def adjust_arm_position(self, dis=0.05):
        # print(f"arm side:{self.arm.side}")
        x_position = self.arm.x_get_position()
        if self.arm.side == "LEFT":
            self.arm.move_x_position(x_position + dis)
        elif self.arm.side == "RIGHT":
            self.arm.move_x_position(x_position - dis)

    def det2pose(self, det, w_r=0.06):
        """
        将检测结果转换为真实世界坐标

        根据检测结果和物体实际宽度，计算物体在真实世界中的坐标和距离。

        参数:
            det: 检测结果，包含 [x, y, w, h]（归一化坐标）
            w_r: 物体实际宽度（米），默认为0.06

        返回:
            tuple: (x坐标, y坐标, 距离)，单位为米
        """
        # r 真实  v 成像  f 焦点
        # rf 真实到焦点的距离  vf 相到焦点的距离
        vf_dis = 1.445
        x_v, y_v, w_v, h_v = det

        rf_dis = vf_dis * w_r / w_v
        x_r = x_v * rf_dis / vf_dis
        y_r = y_v * rf_dis / vf_dis
        return x_r, y_r, rf_dis

    # ====================================================================
    # 机械臂视觉伺服对准(车不动, 纯臂闭环)
    # ====================================================================
    def arm_servo_align(
        self,
        label,
        cx=0.0,
        cy=0.0,
        gains=(0.4, 0.1),
        sign=(1.0, 1.0),
        deadzone=0.05,
        settle=3,
        timeout=7.0,
        max_age=0.3,
    ):
        """机械臂视觉伺服对准: 把目标 label 对齐到画面期望点 (cx, cy)。

        读后台实时缓存(非阻塞), 大臂摆角修画面 cx 误差, X 滑轨伸缩修 cy 误差,
        两轴误差都进死区并连续保持 settle 次即收敛。底盘完全不动。
        适合"车已就位、只需臂就位"的场景(装苗/吸放)。

        参数:
            label:    目标类别(必填), 如 cylinder_1 / cylinder_set
            cx, cy:   期望目标中心(归一化坐标, 默认 (0,0)=画面正中心)
            gains:    (大臂增益, 滑轨增益) 调灵敏度, 标准默认 (0.4, 0.1)
            sign:     (大臂方向符号, 滑轨方向符号) 越对越偏就取反, 默认 (1.0, 1.0)
            deadzone: 两轴误差收敛死区, 默认 0.05
            settle:   误差进死区需连续保持的次数, 默认 3
            timeout:  最大总时长(秒), 默认 7.0
            max_age:  后台缓存最大年龄(秒), 默认 0.3

        返回:
            bool: True=收敛到位, False=超时未到位
        """
        gain_cx, gain_cy = gains
        sign_cx, sign_cy = sign
        end = time.time() + timeout
        hits = 0
        while time.time() < end:
            # 读后台缓存筛目标(多个时取离期望点最近的)
            dets = [
                d
                for d in self.get_realtime_detections(max_age=max_age)
                if d[2] == label
            ]
            if not dets:
                hits = 0
                self.arm.x_speed(0)
                time.sleep(0.02)
                continue
            dets.sort(key=lambda d: (d[4] - cx) ** 2 + (d[5] - cy) ** 2)
            px, py = dets[0][4], dets[0][5]
            e_cx, e_cy = cx - px, cy - py
            if abs(e_cx) < deadzone and abs(e_cy) < deadzone:
                hits += 1
                if hits >= settle:
                    self.arm.x_speed(0)
                    return True
            else:
                hits = 0
                self.arm.set_arm_angle(self.arm.angle + sign_cx * gain_cx * e_cx)
                self.arm.x_speed(sign_cy * gain_cy * e_cy)
            time.sleep(0.03)
        self.arm.x_speed(0)
        return False

    # ====================================================================
    # 底盘视觉对齐(车动, 目标保持画面中心)
    # ====================================================================
    def chassis_align(
        self,
        label,
        cx=0.0,
        cy=0.0,
        kp=(0.10, 0.10),
        sign=(-1.0, 1.0),
        deadband=0.05,
        hold=4,
        v_max=0.12,
        decouple_xy=True,
        timeout=7.0,
        max_age=0.5,
    ):
        """底盘视觉对齐: 移动底盘前后/左右, 把目标 label 对齐到画面期望点 (cx, cy)。

        读后台实时缓存(非阻塞), 水平误差→底盘左右横移(vx), 垂直误差→底盘前后(vy),
        两轴误差都进死区并连续保持 hold 帧即对齐完成。适合"车未就位、需平移对准"
        的场景(放苗前把车对正槽标记 cylinder_set)。

        参数:
            label:    目标类别(必填), 如 cylinder_set / h_tu_dou
            cx, cy:   期望目标中心(归一化坐标, 默认 (0,0)=画面正中心)
            kp:       (x轴增益, y轴增益) 调灵敏度, 默认 (0.1, 0.1)
            sign:     (x轴方向符号, y轴方向符号) 越对越偏就取反
            deadband: 两轴误差收敛死区, 默认 0.05
            hold:     进死区需连续保持的帧数(20Hz), 默认 4
            v_max:    底盘速度上限(m/s), 默认 0.12
            decouple_xy: True=每帧只驱动误差较大单轴(防麦轮 45° 对角打滑);
                        False=两轴同时驱动(旧对角平移)
            timeout:  最大总时长(秒), 默认 7.0
            max_age:  后台缓存最大年龄(秒), 默认 0.5

        返回:
            bool: True=对齐到位(进死区 hold 帧), False=超时/急停
                (目标丢失不再提前放弃, 会一直检索到超时为止)
        """
        end = time.time() + timeout
        in_band = 0
        lost_frames = 0
        last_vx = 0.0
        last_vy = 0.0
        # 麦轮防打滑: 轴滞回, |cx|≈|cy| 时保持上次驱动轴, 避免来回换轴晃
        last_axis = None
        kp_x, kp_y = kp
        sign_x, sign_y = sign
        while True:
            if time.time() > end:
                break
            if getattr(self, "_stop_flag", False):
                break
            # 读后台缓存筛目标(多个时取离期望点最近的)
            dets = [
                d
                for d in self.get_realtime_detections(max_age=max_age)
                if d[2] == label
            ]
            if not dets:
                lost_frames += 1
                in_band = 0
                # 连丢 2 帧后按上次命令反向慢拉回, 避免"找到↔丢帧"来回晃
                if (lost_frames == 2 and (last_vx != 0.0 or last_vy != 0.0)):
                    vx, vy = -last_vx * 0.25, -last_vy * 0.25
                else:
                    vx, vy = 0.0, 0.0
                self.set_velocity(vx, vy, 0.0)
                time.sleep(0.05)
                continue
            dets.sort(key=lambda d: (d[4] - cx) ** 2 + (d[5] - cy) ** 2)
            px, py = dets[0][4], dets[0][5]
            lost_frames = 0
            cx_err, cy_err = cx - px, cy - py

            # P 控制律: decouple_xy 每帧只驱动误差大的单轴(防对角轮打滑)
            if decouple_xy:
                if abs(cy_err) > abs(cx_err) * 1.2 and last_axis == "x":
                    last_axis = "y"
                elif abs(cx_err) > abs(cy_err) * 1.2 and last_axis == "y":
                    last_axis = "x"
                elif last_axis not in ("x", "y"):
                    last_axis = "x" if abs(cx_err) >= abs(cy_err) else "y"
                if last_axis == "x":
                    vx = sign_x * kp_x * cx_err
                    vy = 0.0
                else:
                    vx = 0.0
                    vy = sign_y * kp_y * cy_err
            else:
                vx = sign_x * kp_x * cx_err
                vy = sign_y * kp_y * cy_err

            # v_max 限幅
            vx = max(-v_max, min(v_max, vx))
            vy = max(-v_max, min(v_max, vy))

            if abs(cx_err) < deadband and abs(cy_err) < deadband:
                in_band += 1
                self.set_velocity(0.0, 0.0, 0.0)
                if in_band >= hold:
                    return True
            else:
                in_band = 0
                self.set_velocity(vx, vy, 0.0)
            last_vx, last_vy = vx, vy
            time.sleep(0.05)
        self.set_velocity(0.0, 0.0, 0.0)
        return False
