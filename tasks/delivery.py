import base64
import math
import time
from collections import Counter
from difflib import SequenceMatcher

import cv2

# ==================== 参数 (集中, 方便修改) ====================
# --- 识别 ---
SCAN_ARM        = 95
SCAN_HAND       = -90
SCAN_X          = 0
SCAN_Y          = -0.17          # 识别人名统一高度
N_USERS         = 6
DETECT_RETRY    = 3
OCR_TIMEOUT     = 6.0
OCR_RETRY       = 3
OCR_CROP_SCALES = (1.1, 1.2, 1.4)   # 候选框缩放, 多轮 OCR 取众数

# --- 前后微调扫描 (车体朝向不动, 仅前后 ±10cm 内 5cm 步进) ---
POS_SCAN_MAX    = 0.10      # 前后微调最大位移 (m)
POS_SCAN_STEP   = 0.05      # 步进 5cm
POS_SCAN_FULL   = N_USERS   # 视野内目标齐全即提前停止

# --- 抓取 (单元2从托盘取: 直接 x=0 下降到 y=GRAB_Y 抓取, 不识别蔬菜位置) ---
GRAB_ARM        = 100
GRAB_HAND       = 0
GRAB_X          = 0
GRAB_Y_START    = -0.17
GRAB_Y          = -0.10      # 抓取下降高度 (到达即吸, 不可再往下降)
GRAB_SUCK_DELAY = 0.3
GRAB_Y_LIFT     = -0.15      # 抓取后抬升高度 (直接到放置前携带高度)

# --- 放置 ---
PLACE_ARM          = 93
PLACE_HAND         = -90
PLACE_Y_L1         = -0.10
PLACE_Y_L2         = -0.01
PLACE_Y            = [PLACE_Y_L1, PLACE_Y_L2]
PLACE_X_PUSH       = -0.20     # 前推到名字正下方 (置物架深度)
PLACE_X_BACK       = -0.10     # 释放后回退
PLACE_PAUSE        = 0.2       # 释放后短暂停顿再回退 (气阀切换时间)
PLACE_ALIGN_TIMEOUT = 8.0      # 横向居中名字超时 (s)

# --- 单元间距 ---
UNIT_ADVANCE_M = 0.52   # 单元1 → 单元2 前进 52cm
LANE_FWD_SPEED = 0.1   # 单元1 → 单元2 前视 lane 巡线前进速度 (m/s)

# --- 等待 ---
# ARM_SETTLE: set_arm_pose 本身已是阻塞(等 XY 到位+大臂兜底重发), 这里只留
# 极短的舵机到位缓冲, 不再叠加长时间等待。
ARM_SETTLE  = 0.1


# ==================== 辅助 ====================
def pose(car, arm=None, hand=None, x=None, y=None):
    car.arm.set_arm_pose(x=x, y=y, arm=arm, hand=hand)
    time.sleep(ARM_SETTLE)


def _merge_name_dets(bucket):
    best = {}
    for d in bucket:
        obj = d[1]
        if obj not in best or d[3] > best[obj][3]:
            best[obj] = d
    return list(best.values())


def recognize_name(car, det):
    """两路 OCR (get_det_ocr + 候选框缩放送 LLM) 多轮识别, 返回全部结果列表 (供三级匹配任意命中)."""
    texts = []
    for i in range(OCR_RETRY):
        try:
            text = car.get_det_ocr(det, label="name", time_out=OCR_TIMEOUT)
            if text:
                text = str(text).strip().strip(" ,.!?。，！？、：:;；()（）《》[]")
                if text:
                    texts.append(text)
        except Exception:
            pass
        try:
            img = car.side_image
            scale = OCR_CROP_SCALES[i % len(OCR_CROP_SCALES)]
            x1, y1, x2, y2 = car._bbox_to_pixel(det[4:], img.shape, scale=scale)
            if x2 > x1 and y2 > y1:
                crop = img[y1:y2, x1:x2]
                _, enc = cv2.imencode(".jpg", crop)
                b64 = base64.b64encode(enc.tobytes()).decode("utf-8")
                resp = car.image_analysis.client.chat.completions.create(
                    model=car.image_analysis.image_model,
                    messages=[{"role": "user", "content": [
                        {"type": "text",
                         "text": "识别图片中的中文人名。只输出纯文本的姓名两到四个字,不要标点、解释和其他内容。若图中无清晰文字输出空字符串。"},
                        {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{b64}"}},
                    ]}],
                    top_p=0.1,
                )
                text = resp.choices[0].message.content.strip().strip(
                    " ,.!?。，！？、：:;；()（）《》[]\n\t\"'")
                bad = ("无法识别", "识别", "无法", "无文字", "看不清", "不是", "未识别", "None", "none", "图片")
                if text and not any(b in text for b in bad) and len(text) >= 2:
                    texts.append(text)
        except Exception as e:
            print(f"    识别第{i+1}回合异常: {e}")
    return texts


# ==================== Phase B: 扫描6个人名 (前后微调, 不扭角) ====================
def _scan_at_pos(car, collected, obj_idx):
    """在当前位置直接扫描. 新人名: OCR+添加; 已有人名: 重扫合并texts (每位置每牌只扫一次)."""
    bucket = []
    for _ in range(DETECT_RETRY):
        dets = car.get_detection_results(sort_pos=(0, 0.5), limit_x=1.0)
        bucket.extend(d for d in dets if d[2] == "name")
        time.sleep(0.05)

    ocr_done = set()  # 每位置每牌只扫一次
    for d in _merge_name_dets(bucket):
        obj_id = d[1]
        if obj_id in ocr_done:
            continue
        ocr_done.add(obj_id)
        t0 = time.monotonic()
        new_texts = recognize_name(car, d)
        if obj_id in obj_idx:
            idx = obj_idx[obj_id]
            collected[idx][0].extend(new_texts)
            tag = f"重扫合并→{len(collected[idx][0])}条"
        else:
            collected.append([new_texts, d, 0.0])
            obj_idx[obj_id] = len(collected) - 1
            tag = f"新人名 @x_c={d[4]:.3f} y_c={d[5]:.3f}"
        display = Counter(new_texts).most_common(1)[0][0] if new_texts else None
        print(f"    '{display}' {tag} 耗时{time.monotonic()-t0:.1f}s")
    print(f"    已收 {len(collected)}/{N_USERS}")


def scan_names(car, unit_tag):
    """y=SCAN_Y 高度, 前后微调扫描 (5cm步进, 最多 ±POS_SCAN_MAX).
    车体朝向不动, 仅前后 ±10cm 内 5cm 步进; 已有人名在新位置重扫合并texts;
    视野内已收齐 N_USERS 即提前停止.
    """
    print(f"\n=== [{unit_tag}] 扫描人名 (y={SCAN_Y:.2f}m, 前后微调±{POS_SCAN_MAX*100:.0f}cm, 步 {POS_SCAN_STEP*100:.0f}cm) ===")
    pose(car, arm=SCAN_ARM, hand=SCAN_HAND, x=SCAN_X, y=SCAN_Y)

    obj_idx, collected = {}, []
    home_g = car.get_global_pose()

    offsets = [0.0]
    for i in range(1, int(POS_SCAN_MAX / POS_SCAN_STEP) + 1):
        offsets.append(round( i * POS_SCAN_STEP, 3))
        offsets.append(round(-i * POS_SCAN_STEP, 3))

    for oi, offset in enumerate(offsets):
        if offset != 0.0:
            car.go_to_global_pose(home_g)
            print(f"  位置 {oi+1}/{len(offsets)} 前后微调 {offset*100:+.0f}cm 补扫")
            car.move_for([offset, 0.0, 0.0])   # 阻塞闭环, 到位即停
            time.sleep(0.1)
        else:
            print(f"  位置 {oi+1}/{len(offsets)} 原位扫描")
        _scan_at_pos(car, collected, obj_idx)
        if len(collected) >= POS_SCAN_FULL:
            print(f"  视野内已收齐 {POS_SCAN_FULL} 名, 提前结束微调")
            break

    car.go_to_global_pose(home_g)
    n = len(collected)
    print(f"  {'警告: 仅' if n < N_USERS else '识别完成: '}{n}/{N_USERS} 个名字")
    car.beep()
    return collected


# ==================== Phase C: 匹配目标 + 保底 ====================
def find_target(name_results, target):
    """三级匹配 (①②级只要任意一次识别满足即命中):
    ① 完全相同 ② 至少共享1字(取共享最多) ③ 相似度最高(保底).
    返回 (det, layer_idx, yaw_deg) 或 None.
    """
    # 按 y_c 排序: 前 3 = 上层 (layer 0), 后 3 = 下层 (layer 1)
    ordered = sorted(name_results, key=lambda x: x[1][5])
    target_chars = set(target)

    # ① 精确匹配: 任意一次识别与目标完全相同
    for i, (texts, det, yaw_deg) in enumerate(ordered):
        if texts and any(t == target for t in texts):
            return det, (0 if i < 3 else 1), yaw_deg

    # ② ③: 取每个名字所有识别结果中的最佳(共享字优先, 同数取相似度高)
    best = None
    best_key = (-1, -1.0)
    for i, (texts, det, yaw_deg) in enumerate(ordered):
        if not texts:
            continue
        key = max(
            (len(target_chars & set(t)), SequenceMatcher(None, target, t).ratio())
            for t in texts
        )
        if key > best_key:
            best_key = key
            best = (det, (0 if i < 3 else 1), yaw_deg, texts)

    if best is None:
        return None
    det, layer, yaw_deg, texts = best
    display = Counter(texts).most_common(1)[0][0]
    if best_key[0] > 0:
        print(f"  [匹配] 精确未命中, 选与目标共享{best_key[0]}字的人名: '{display}'")
    else:
        print(f"  [保底] 无共享字, 选最相似的人名: '{display}' (相似度={best_key[1]:.2f})")
    return det, layer, yaw_deg


# ==================== Phase D: 抓取蔬菜 (直接 x=0 下降抓取, 不识别位置) ====================
def grab_vegetable(car):
    """摆位姿 → 关泵 → 在 x=GRAB_X 直接下降到 y=GRAB_Y 吸取 → 抬升到 GRAB_Y_LIFT (放置前携带高度)."""
    pose(car, arm=GRAB_ARM, hand=GRAB_HAND, x=GRAB_X, y=GRAB_Y_START)
    car.arm.grasp(False)
    time.sleep(0.05)
    print(f"  在 x={GRAB_X} 直接下降到 y={GRAB_Y:.4f}m 吸取")
    car.arm.move_y_position(GRAB_Y)      # 阻塞: 到位即已物理完成
    car.arm.grasp(True)
    time.sleep(GRAB_SUCK_DELAY)          # 气泵吸住需时间
    car.arm.move_y_position(GRAB_Y_LIFT)  # 阻塞
    print(f"  抓取完毕, 已抬到 y={GRAB_Y_LIFT:.4f}m (携带高度)")


# ==================== Phase E: 放置蔬菜 ====================
def place_vegetable(car, target_det, layer_idx, yaw_det):
    print(f"  目标: 第{layer_idx+1}层, 原扫角 yaw={yaw_det:+.1f}°, x_c={target_det[4]:.3f}, y_c={target_det[5]:.3f}")
    pose(car, arm=PLACE_ARM, hand=PLACE_HAND, x=SCAN_X, y=SCAN_Y)

    # scan 期间车体朝向不动, 全程 yaw=0; 直接在 yaw=0 帧横向居中名字, 不再转动车体 ——
    # 转动会破坏居中, 蔬菜落不到正下方.
    home_g = car.get_global_pose()   # 记原位: 放置后回位, 单元1 回位后才前进 52cm

    print(f"  对齐名字 (横向居中, lock=True, 超时 {PLACE_ALIGN_TIMEOUT}s)")
    car.move_to_detection_target(
        label="name", sort_pos=(target_det[4], target_det[5]),
        delta_y=None, time_out=PLACE_ALIGN_TIMEOUT, lock=True,
    )   # 阻塞: 对齐到位即停

    # 下降到该层高度, 前推到名字正下方, 释放, 回退 (均为阻塞运动)
    car.arm.move_y_position(PLACE_Y[layer_idx])
    car.arm.move_x_position(PLACE_X_PUSH)
    car.arm.grasp(False)
    time.sleep(PLACE_PAUSE)          # 气阀释放切换时间
    car.arm.move_x_position(PLACE_X_BACK)
    print(f"  放置完成 (第{layer_idx+1}层, y={PLACE_Y[layer_idx]})")

    # 回到对齐前原位: 撤销横向居中产生的位移 (闭环到全局位姿, 对平移/朝向漂移都鲁棒);
    # 单元1 回位后由主流程前进 52cm 到单元2, 保证起点准确.
    car.go_to_global_pose(home_g)
    print(f"  已回原位 {car.get_global_odometry_str()}")


# ==================== 单元流程 (单元1/单元2 复用) ====================
def run_unit(car, tag, target, grab_first):
    """扫描 → 匹配 → (可选抓取) → 放置. match 为 None 时跳过放置, 不影响后续单元."""
    print("\n" + "=" * 40)
    print(f"           {tag} 开始")
    print("=" * 40)

    names = scan_names(car, tag)
    match = find_target(names, target)
    if match is None:
        print(f"[{tag}] 保底失败: 无任何可用人名, 跳过放置")
        print("  已识别: " + ", ".join(
            Counter(t).most_common(1)[0][0] if t else "?" for t, _, _ in names))
        return

    det, layer, yaw = match
    print(f"  [{tag}] 匹配: 第{layer+1}层 '{target}' (原扫角 yaw={yaw:+.1f}°)")
    if grab_first:
        print(f"\n[{tag}] 从托盘抓取车上蔬菜")
        grab_vegetable(car)
    print(f"\n[{tag}] 放置蔬菜")
    place_vegetable(car, det, layer, yaw)
    print(f"\n[{tag}] 完成 ✓")
    car.beep()


def advance_to_unit2(car):
    """前视 lane 巡线前进 UNIT_ADVANCE_M 米; 起步前先原地 yaw 复位清滞回, 让 lane 在已对正位姿下锁线."""
    print(f"\n=== 前进 {UNIT_ADVANCE_M*100:.0f}cm 到单元2 (前视 lane 巡线, speed={LANE_FWD_SPEED}m/s) ===")
    print("  [巡线起步前] yaw 复位: +0.5° → −0.5° → 0° (校准零点)")
    car.move_for([0.0, 0.0, math.radians(+0.5)])
    time.sleep(0.1)
    car.move_for([0.0, 0.0, math.radians(-1.0)])
    time.sleep(0.1)
    car.move_for([0.0, 0.0, math.radians(+0.5)])
    time.sleep(0.1)
    car.lane_dis_offset(speed=LANE_FWD_SPEED, dis_hold=UNIT_ADVANCE_M)
    print(f"  lane 巡线前进完成, 累计 {UNIT_ADVANCE_M*100:.0f}cm")


# ==================== 主流程 ====================
def run(car, order_list=None):
    """编排器入口: order_list 来自上游 ordering 任务, 每项含 name/goods/address(1或2).
    流程: 单元1(抓取→扫描→放置) → 前进52cm → 单元2(抓取→扫描→放置).
    """
    if not order_list:
        print("[delivery] 无订单, 退出")
        return

    target_u1 = next((o.get("name") for o in order_list if o.get("address") == 1), None)
    target_u2 = next((o.get("name") for o in order_list if o.get("address") == 2), None)
    if not target_u1 or not target_u2:
        print(f"[delivery] 订单人名不完整: 单元1={target_u1}, 单元2={target_u2}, 退出")
        return

    try:
        run_unit(car, "单元1", target_u1, grab_first=True)
        advance_to_unit2(car)
        run_unit(car, "单元2", target_u2, grab_first=True)

        print("\n" + "=" * 40)
        print("      全部完成 (单元1 + 单元2) ✓")
        print("=" * 40)
        car.beep()
    except KeyboardInterrupt:
        print("\n急停")
    finally:
        car.arm.grasp(False)
