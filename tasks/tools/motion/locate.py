# -*- coding: utf-8 -*-
"""目标定位与机械臂逼近(LocateMixin): 检测目标对齐、世界坐标换算(从 motion.py 拆分而来)。"""
import math
import time

import cv2
from typing import Union

from ..coords import norm_box_to_pixel
from smartcar import PID, logger
from smartcar.whalesbot.tools import CountRecord, get_yaml


# ================================================================
# 大臂档位 → 前后符号 (双情况判定; 实车 --live 验证过, 测试见 scripts/test_chassis_align.py)
# ================================================================
def resolve_fwd_sign(arm_angle, calibrated=None):
    """大臂角度档位 → 画面横向误差(cx)驱动车前后(vx)的符号.

    竖拍(大臂≤-45°): 目标在画面左 → 车前进, 右 → 车后退 → +1
    横拍(大臂≥+45°): 目标在画面左 → 车后退, 右 → 车前进 → -1
    中间区(|arm|<45°)方向无定论 → 0.0(不自动驱动前后, 需人工给 sign)。
    calibrated: 现场人工确认过的符号, 非 None 时优先返回。
    """
    if calibrated is not None:
        return float(calibrated)
    if arm_angle <= -45.0:
        return 1.0
    if arm_angle >= 45.0:
        return -1.0
    return 0.0


def fwd_vx(arm_angle, cx_err, kp_y=0.22, sign_y=None):
    """按大臂档位符号算车前后速度 vx = sign_y * kp_y * cx_err (镜像 chassis_align)."""
    if sign_y is None:
        sign_y = resolve_fwd_sign(arm_angle)
    return sign_y * kp_y * cx_err


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
        time_out=6.0,
        sort_pos=(0, 0),
        num=0,
        score_thresh=None,
        lock=False,
        min_score=0.0,
        select_range=None,
    ):
        """
        前往目标位置

        参数:
            cls_id : 指定检测目标的 cls_id，默认None为距离中心最近的目标
            time_out: 设置超时时间
            包含目标检测信息的列表，格式为 [cls_id, obj_id,label, score, x_c, y_c, w, h]
            lock: 锁定目标(按位置连续性跟踪)。对齐过程中画面偏移会改变距 sort_pos
                  最近的动物, 选中会切换; 开启后每帧按距上次选中目标的位置排序选中,
                  目标锁定不再漂移。
        """
        time_stop = time.time() + time_out
        x_count = CountRecord(3)
        y_count = CountRecord(3)
        lock_pos = None  # 锁定目标位置 (x_c, y_c)
    
       
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
            ki_x = 0.06

        pid_x = PID(kp_x, ki_x)
        pid_x.output_limits = (-0.15, 0.15)
        pid_x.setpoint = delta_x
        while True:
            if self._stop_flag:
                self.set_velocity(0, 0, 0)
                self.arm.x_speed(0)
                return -1, "None"

            dets = self.get_detection_results(sort_pos=sort_pos, score_thresh=score_thresh)

            if label is not None:
                dets = [item for item in dets if item[2] == label]

            if min_score > 0:
                dets = [item for item in dets if item[3] > min_score]
                
            if select_range is not None:
                # 位置硬过滤: 只接受 x_c 距目标站位 delta_x 在窗口内的检测,
                # 剔除偏离站位的干扰(如已被击倒的动物残骸)
                dets = [item for item in dets if abs(item[4] - delta_x) <= select_range]    

            if len(dets) > num:
                det = dets[num]
                # 锁定目标: 每帧更新为其当前位置, 后续帧按距它排序选中
                if lock_pos is None:
                    print(f"[锁定] animal score={det[3]:.2f} "
                            f"pos=({det[4]:+.3f},{det[5]:+.3f})")
                lock_pos = (det[4], det[5])
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

                flag_x = x_count(abs(err_x) < 0.04)
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
        settle=6,
        lock=5,
        prefer_left=False,
        prefer_right=False,
        timeout=7.0,
        max_age=0.3,
        min_score=0.0,
        speed_cap=None,
        persist_track=True,
        x_check=False,
        auto_sign=False,
        debug=False,
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
            lock:     追踪前目标须累计出现的帧数(累计计数, 不因单帧漏检重数, 防闪帧/卡死), 默认 5
            prefer_left: True=目标多时优先锁定画面最左侧那个(px 最小), 默认 False。
                        prefer_right: True=目标多时优先锁定画面最右侧那个(px 最大), 默认 False。
                        两者互斥, 同 True 时 prefer_left 生效。按任务臂位选择:
                        左臂对齐(抓 cylinder_set)用 prefer_left, 右臂对齐(抓 cylinder_1/2/3)用 prefer_right。
            timeout:  最大总时长(秒), 默认 7.0
            max_age:  后台缓存最大年龄(秒), 默认 0.3
            min_score: 目标置信度下限(score∈[0,1]), 低于此分的框不算目标; 默认 0.0 不过滤
            speed_cap: 滑轨速度上限(m/s), 非 None 时启用"远端满速快走": 比例输出钳到 ±speed_cap,
                        误差大时不再因 Kp 小慢吞吞, 只小误差才进比例段。默认 None=沿用原 Kp*err。
            persist_track: True=目标多帧稳定时优先续锁上一帧选中的目标(滞回, 防多目标来回跳),
                        仅当旧目标消失才按 prefer/最近规则换目标。默认 True。
            x_check: True=滑轨命令加"回读确认": 发速度后隔帧读 x_get_position() 验证是否按
                        预期动; 两次读都没动而命令非零则判定该帧丢失, 重发一次。默认 False。
                        滑轨有编码器可回读, 是判断"命令是否落地"的唯一轴, 兼作 X 顶墙侦察。
            auto_sign: True=进入闭环前做一次试探性滑轨微动, 读回位移方向, 与 sign_cy 预期不一致
                        时自动翻转滑轨符号并告警(换车/换相机后免人工重标该轴)。默认 False。
            debug:    逐帧打印 px/py/误差/输出, 用于现场定方向符号(默认 False)

        返回:
            bool: True=收敛到位, False=超时未到位
        """
        gain_cx, gain_cy = gains
        sign_cx, sign_cy = sign
        t0 = time.monotonic()  # 单调钟, 不受 NTP 校时跳变影响, 避免对齐提前/超时退出
        end = t0 + timeout
        hits = 0
        lock_cnt = 0
        locked = False
        seen_once = False
        # 滞回状态: 上一帧选中的目标位置(px), 下一帧若仍可见则优先续锁, 防多目标来回跳
        last_px = None
        # x_check 回读确认状态
        x_last_pos = None      # 上一次发命令前的滑轨位置
        x_cmd_sent = False     # 本帧是否发了非零滑轨速度
        x_pending_retry = 0     # 待重发的滑轨速度(m/s), 0=无
        if auto_sign:
            # 试探性微动: 发一个已知方向的小速度一小段, 读回位移方向与 sign_cy 比对。
            # 滑轨能回读(x_get_position), 是唯一能自证方向的轴; 大臂 PWM 舵机无回读, 只能人工。
            probe_v = 0.05
            p0 = self.arm.x_get_position()
            self.arm.x_speed_async(probe_v)
            time.sleep(0.2)
            self.arm.x_speed(0)
            time.sleep(0.05)
            dp = self.arm.x_get_position() - p0
            if abs(dp) < 1e-4:
                print(f"[伺服] 滑轨自检: 微动无位移(疑似丢帧/顶墙), 未翻转符号, sign_cy 维持 {sign_cy}")
            else:
                want = probe_v if sign_cy > 0 else -probe_v
                if (dp > 0) != (want > 0):
                    sign_cy = -sign_cy
                    print(f"[伺服] 滑轨自检: 实测方向 {dp:+.4f} 与预期反, sign_cy {sign} → {sign_cy}")
                else:
                    print(f"[伺服] 滑轨自检: 方向正确 (dp={dp:+.4f})")
        print(f"[伺服] 对齐 {label}: 期望点({cx},{cy}) deadzone={deadzone} 锁定{lock}帧 超时{timeout}s")
        while time.monotonic() < end:
            # 读后台缓存筛目标(多个时取离期望点最近的)
            dets = [
                d
                for d in self.get_realtime_detections(max_age=max_age)
                if d[2] == label and d[3] >= min_score
            ]
            if not dets:
                # 缺帧不清零 lock_cnt(只不再累计), 避免偶发漏检导致永远锁不定;
                # 已达成的锁定保持。hits(收敛计数)仍清零。
                locked = True  # 已有锁定在缺帧时保持不降级
                hits = 0
                self.arm.x_speed(0)
                time.sleep(0.02)
                continue
            seen_once = True
            # 锁定: 目标累计出现 lock 帧后开始追踪(不因单帧丢失重数, 避免卡死);
            # 未锁定阶段臂/滑轨不动。
            lock_cnt += 1
            if lock_cnt < lock:
                self.arm.x_speed(0)
                time.sleep(0.02)
                continue
            if not locked:
                locked = True
                print(f"  [{label}] 已锁定({lock_cnt}帧), 开始追踪")
            # 目标挑选: prefer_right 优先画面最右(px 最大), prefer_left 优先最左(px 最小),
            # 否则默认取离期望点最近的。
            # persist_track 滞回: 只要上一帧选中的目标仍在缓存里, 优先续锁它(即使它不再是
            # 最左/最右/最近), 只有它消失才换目标——防止多目标下在几个目标间来回跳导致 hits 归零。
            if persist_track and last_px is not None:
                prev = [d for d in dets if abs(d[4] - last_px) < 0.02]
                if prev:
                    dets = prev
            if prefer_right and not prefer_left:
                dets.sort(key=lambda d: -d[4])
            elif prefer_left:
                dets.sort(key=lambda d: d[4])
            else:
                dets.sort(key=lambda d: (d[4] - cx) ** 2 + (d[5] - cy) ** 2)
            px, py = dets[0][4], dets[0][5]
            last_px = px
            e_cx, e_cy = cx - px, cy - py
            if abs(e_cx) < deadzone and abs(e_cy) < deadzone:
                hits += 1
                if hits >= settle:
                    self.arm.x_speed(0)
                    print(f"[伺服] {label} 收敛: 用时 {time.monotonic() - t0:.2f}s")
                    return True
            else:
                hits = 0
                # 滑轨速度: 原比例输出, 可选 speed_cap 做"远端满速快走"(误差大时不受 Kp 小限制)
                vx = sign_cy * gain_cy * e_cy
                if speed_cap is not None:
                    vx = max(-speed_cap, min(speed_cap, vx))
                # x_check 回读确认: 滑轨有编码器可回读, 是唯一能自证"命令是否落地"的轴。
                # 上帧发了非零速度而本次读回位置几乎未变 → 判定该帧丢失 → 重发上帧速度。
                if x_check:
                    cur = self.arm.x_get_position()
                    if x_cmd_sent and x_last_pos is not None and abs(x_pending_retry) > 1e-6:
                        if abs(cur - x_last_pos) < 2e-4:
                            self.arm.x_speed_async(x_pending_retry)
                            if debug:
                                print(f"  [{label}] 滑轨帧丢失重发 v={x_pending_retry:+.4f}")
                    x_pending_retry = vx
                    x_last_pos = cur
                # 异步发帧(不阻塞等回包): XY 舵机同一条 MC602 总线, 同步发帧
                # 在电机刷屏时极易被挤掉(实测手爪/大臂概率不动); 异步走 submit
                # 排队, 掉帧率大降。
                self.arm.set_arm_angle_async(self.arm.angle + sign_cx * gain_cx * e_cx)
                self.arm.x_speed_async(vx)
                x_cmd_sent = abs(vx) > 1e-6
                if debug:
                    print(f"  [{label}] px={px:+.3f} py={py:+.3f} e_cx={e_cx:+.3f} e_cy={e_cy:+.3f}"
                          f" → 臂{sign_cx * gain_cx * e_cx:+.2f}° 滑轨{vx:+.4f}m/s")
            time.sleep(0.03)
        self.arm.x_speed(0)
        print(f"[伺服] {label} 超时未收敛: 用时 {time.monotonic() - t0:.2f}s"
              f" ({'全程未见目标' if not seen_once else '目标出现过但未进死区'})")
        return False

    # ====================================================================
    # 底盘视觉对齐(车动, 目标保持画面中心)
    # ====================================================================
    def chassis_align(
        self,
        label,
        cx=0.0,
        cy=0.0,
        kp=(0.15, 0.08),
        sign=None,
        deadband=0.03,
        hold=6,
        v_max=0.12,
        v_min=0.005,
        decouple_xy=True,
        timeout=7.0,
        max_age=0.5,
        prefer_left=False,
        prefer_right=False,
    ):
        """底盘视觉对齐: 移动底盘前后/左右, 把目标 label 对齐到画面期望点 (cx, cy)。

        读后台实时缓存(非阻塞), 交叉映射: 画面横向误差 cx_err→车前后 vx、画面纵向误差
        cy_err→车左右 vy (侧视相机竖拍, 画面横向=场地纵深、画面纵向=场地横向)。
        两轴误差都进死区并连续保持 hold 帧即对齐完成。适合"车未就位、需平移对准"
        的场景(放苗前把车对正槽标记 cylinder_set)。

        参数:
            label:    目标类别(必填), 如 cylinder_set / h_tu_dou
            cx, cy:   期望目标中心(归一化坐标, 默认 (0,0)=画面正中心)
            kp:       (车左右增益, 车前后增益) 调灵敏度, 默认 (0.15, 0.08)
            sign:     (车左右横移符号, 车前后符号); 交叉映射: 画面横向误差驱动车前后(vx,
                      正=前进)、画面纵向误差驱动车左右(vy)。None=自动: 前后符号按当前
                      大臂档位 resolve_fwd_sign(arm.angle) 定(竖拍≤-45°→+1 目标左前进;
                      横拍≥45°→-1 目标左后退; 中间区→0 前后不动), 横向符号无自动值取
                      -1.0。显式传 (sign_x, sign_y) 时完全用传入值。
            deadband: 两轴误差收敛死区, 默认 0.03
            hold:     进死区需连续保持的帧数(20Hz), 默认 6
            v_max:    底盘速度上限(m/s), 默认 0.12
            v_min:    输出死区(|v|<该值置0), 默认 0.005; 调大防抖/调小防静差
            decouple_xy: True=每帧只驱动误差较大单轴(防麦轮 45° 对角打滑);
                        False=两轴同时驱动(旧对角平移)
            timeout:  最大总时长(秒), 默认 7.0
            max_age:  后台缓存最大年龄(秒), 默认 0.5
            prefer_left: True=目标多时优先锁定画面最左侧那个(px 最小), 默认 False。
            prefer_right: True=目标多时优先锁定画面最右侧那个(px 最大), 默认 False。
                两者互斥, 同 True 时 prefer_left 生效。语义同 arm_servo_align。

        返回:
            bool: True=对齐到位(进死区 hold 帧), False=超时/急停
                (目标丢失不再提前放弃, 会一直检索到超时为止)
        """
        t0 = time.monotonic()  # 单调钟, 不受 NTP 校时跳变影响, 避免对齐提前/超时退出
        end = t0 + timeout
        in_band = 0
        lost_frames = 0
        last_vx = 0.0
        last_vy = 0.0
        # 麦轮防打滑: 轴滞回, |cx|≈|cy| 时保持上次驱动轴, 避免来回换轴晃
        last_axis = None
        kp_x, kp_y = kp
        if sign is None:
            # 双情况自动判: 前后符号按当前大臂档位; 横向符号无自动值, 取 -1.0
            sign_x, sign_y = -1.0, resolve_fwd_sign(self.arm.angle)
        else:
            sign_x, sign_y = sign
        print(f"[底盘] 对齐 {label}: 期望点({cx},{cy}) deadband={deadband} 超时{timeout}s")
        while True:
            if time.monotonic() > end:
                print(f"[底盘] {label} 超时未对齐: 用时 {time.monotonic() - t0:.2f}s")
                break
            if getattr(self, "_stop_flag", False):
                print(f"[底盘] {label} 急停中断: 用时 {time.monotonic() - t0:.2f}s")
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
                if (lost_frames == 5 and (last_vx != 0.0 or last_vy != 0.0)):
                    vx, vy = -last_vx * 0.25, -last_vy * 0.25
                else:
                    vx, vy = 0.0, 0.0
                self.set_velocity(vx, vy, 0.0)
                time.sleep(0.05)
                continue
            if prefer_right and not prefer_left:
                dets.sort(key=lambda d: -d[4])
            elif prefer_left:
                dets.sort(key=lambda d: d[4])
            else:
                dets.sort(key=lambda d: (d[4] - cx) ** 2 + (d[5] - cy) ** 2)
            px, py = dets[0][4], dets[0][5]
            lost_frames = 0
            cx_err, cy_err = cx - px, cy - py

            # 交叉映射(用户明确要求): 侧视相机竖拍, 画面横向=场地纵深、画面纵向=场地横向。
            # set_velocity(x=车前后, y=车横向), 所以 画面横向误差 cx_err 驱动车前后(x-slot),
            # 画面纵向误差 cy_err 驱动车横向(y-slot)。kp=(左右增益, 前后增益),
            # sign=(左右符号, 前后符号)。(旧实现把两轴算反: 前后被 cy_err 驱动,
            # 横向被 cx_err 驱动, 实车表现为"横移", 已修正)
            # P 控制律: decouple_xy 每帧只驱动误差大的单轴(防对角轮打滑)
            if decouple_xy:
                if abs(cy_err) > abs(cx_err) * 1.2 and last_axis == "x":
                    last_axis = "y"
                elif abs(cx_err) > abs(cy_err) * 1.2 and last_axis == "y":
                    last_axis = "x"
                elif last_axis not in ("x", "y"):
                    last_axis = "x" if abs(cx_err) >= abs(cy_err) else "y"
                if last_axis == "x":          # 车前后 ← 画面横向误差
                    vx = sign_y * kp_y * cx_err
                    vy = 0.0
                else:                         # 车横向 ← 画面纵向误差
                    vx = 0.0
                    vy = sign_x * kp_x * cy_err
            else:
                vx = sign_y * kp_y * cx_err
                vy = sign_x * kp_x * cy_err

            # v_max 限幅
            vx = max(-v_max, min(v_max, vx))
            vy = max(-v_max, min(v_max, vy))

            if abs(vx) < v_min:
                vx = 0.0
            if abs(vy) < v_min:
                vy = 0.0

            if abs(cx_err) < deadband and abs(cy_err) < deadband:
                in_band += 1
                self.set_velocity(0.0, 0.0, 0.0)
                if in_band >= hold:
                    print(f"[底盘] {label} 收敛: 用时 {time.monotonic() - t0:.2f}s")
                    return True
            else:
                in_band = 0
                self.set_velocity(vx, vy, 0.0)
            last_vx, last_vy = vx, vy
            time.sleep(0.05)
        self.set_velocity(0.0, 0.0, 0.0)
        return False
