#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import time

STEP = 0.14                  # 相邻目标板间距 (m)
ROW_END = 0.42               # 4 块板总宽 = 3 * STEP, 射完补偿推进到排尾

SHOOT_DISTANCE_MIN_M = 0.45  # 达标距离窗口下限 (m)
SHOOT_DISTANCE_MAX_M = 1.00  # 达标距离窗口上限 (m)
TARGET_BOARD_WIDTH_M = 0.08  # 目标板实际宽度 (m)
YAW_HFOV_DEG = 70.0          # 侧摄像头水平视场角 (度, 估算值)
EDGE_MARGIN = 0.08           # 完整入画: bbox 左缘 xc-wn/2 需 >= -1+该余量
SCORE_THRESH = 0.9           # 检测置信度门槛

ALIGN_SPEED = 0.1            # 距离对齐时的前进/后退速度
ALIGN_TIME_OUT = 2.0         # 单次对齐超时 (s)
LOCK_COUNT = 2               # 距离连续达标帧数 (防抖)


def width_to_distance_m(wn):
    """bbox 归一化宽度 -> 目标板距离 (m): D = W / (2*wn*tan(H/2))."""
    return TARGET_BOARD_WIDTH_M / (2.0 * wn * math.tan(math.radians(YAW_HFOV_DEG) / 2.0))


def pick_front_target(dets):
    """选最前「完整入画」目标: 左缘不贴边截断, 再取 xc 最小 (最左) 者.

    返回检测项 [cls_id, obj_id, label, score, x_c, y_c, w, h] 或 None.
    """
    cands = [d for d in dets if d[4] - d[6] / 2.0 >= -1.0 + EDGE_MARGIN]
    if not cands:
        return None
    return min(cands, key=lambda d: d[4])


def run(car, animal_list=None):
    """依次对齐击发所有确认害虫 (animal_list 中值为 0 的板位)."""
    if animal_list is None:
        animal_list = [0, 0, 0, 0]  # 未传识别结果 -> 视为全部为害虫

    targets = [i for i, v in enumerate(animal_list) if v == 0]
    if not targets:
        print("[shooting] 无确认害虫, 跳过射击", flush=True)
        return

    car.arm.set_arm_pose(arm="LEFT", hand="UP", x=-0.25, y=-0.04)

    def align():
        """距离窗口对齐: 最前完整入画目标反推距离落进窗口即停."""
        ok = 0
        deadline = time.time() + ALIGN_TIME_OUT
        while time.time() < deadline:
            dets = car.get_detection_results(score_thresh=SCORE_THRESH)
            det = pick_front_target(dets)
            if det is None:
                car.set_velocity(0, 0, 0)
                time.sleep(0.1)
                continue
            d_m = width_to_distance_m(det[6])  # 用未 clamp 的原始 bbox 宽度
            if SHOOT_DISTANCE_MIN_M <= d_m <= SHOOT_DISTANCE_MAX_M:
                car.set_velocity(0, 0, 0)
                ok += 1
                if ok >= LOCK_COUNT:
                    print(f"[shooting] 距离达标 {d_m:.2f}m (xc={det[4]:.2f}), 准备击发",
                          flush=True)
                    return True
                time.sleep(0.05)
                continue
            ok = 0
            speed = ALIGN_SPEED if d_m > SHOOT_DISTANCE_MAX_M else -ALIGN_SPEED
            car.set_velocity(speed, 0, 0)  # 太远前进, 太近后退
            time.sleep(0.05)
        car.set_velocity(0, 0, 0)
        print("[shooting] 距离对齐超时, 按当前姿态击发", flush=True)
        return False

    align()  # 先对齐排首可见板, 建立射击基线
    prev = 0
    for idx in targets:
        car.lane_dis_offset(speed=0.2, dis_hold=(idx - prev) * STEP)  # 推进到本板
        align()                                                        # 对准
        time.sleep(0.2)
        car.shooting(pulse_seconds=0.23)                                                 # 击发 (自带蜂鸣提示)
        time.sleep(0.2)
        prev = idx

    # 射完推进到排尾, 为后续巡线留位
    car.lane_dis_offset(speed=0.2, dis_hold=ROW_END - targets[-1] * STEP)
