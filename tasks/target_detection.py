import threading
import time

# 巡线巡航速度(m/s): 纯直线经行(开环, 不看车道, 无转向校正), 恒速扫过 4 只 animal
CRUISE_SPEED = 0.20
# 动物识别置信度阈值: 置信度大于此值的 animal 才送大模型
ANIMAL_CONF = 0.85
MAX_ANIMALS = 4  # 需识别动物数
# 触发采集的画面中央窗口(|x_c|<=该值): 目标经过画面中央时裁剪最完整, 避免切边缘残框
CAPTURE_WINDOW = 0.30


def _collect_loop(car, captures, alive):
    """
    后台线程: 巡线经行中只快速采集 animal 裁剪帧(毫秒级, 不调大模型).
    状态机: 中央窗口内有 animal 且未捕获当前只 -> 捕获存帧; 该只离开中央
    窗口(或画面)后才预备捕获下一只; 同一只只捕获一次(armed 置 False)。
    """
    armed = True
    while (
        alive[0]
        and not getattr(car, "_stop_flag", False)
        and len(captures) < MAX_ANIMALS
    ):
        dets = [
            d
            for d in car.get_realtime_detections(max_age=0.3)
            if d[2] == "animal" and d[3] >= ANIMAL_CONF
        ]
        a = min(dets, key=lambda d: abs(d[4])) if dets else None
        if a is None:
            armed = True
        elif armed and abs(a[4]) <= CAPTURE_WINDOW:
            frame = car.cap_side.frame
            if frame is None:
                time.sleep(0.02)
                continue
            captures.append((a, frame.copy()))
            print(f"[采集] 第 {len(captures)}/{MAX_ANIMALS} 只 (x_c={a[4]:.2f})")
            car.beep()
            armed = False
        elif not armed and abs(a[4]) > CAPTURE_WINDOW:
            armed = True
        time.sleep(0.02)


def _infer_all(car, captures, results):
    """停车后统一送大模型: 逐只裁剪识别, 结果入 results(害=0/益=1, 失败=0)."""
    for i, (det, frame) in enumerate(captures):
        res = car.animal_image_analysis(det=det, image=frame)
        results.append(0 if res is None else res)
        print(
            f"[识别] 第 {i + 1}/{len(captures)} 只: "
            + ("害" if res == 0 else "益" if res == 1 else f"{res}")
        )


def run(car) -> list:
    # 每元素 害/益: 害=0 需击打, 益=1
    # 机械臂位姿一次调好, 全程保持(不再为识别逐只停下来)
    car.arm.set_arm_pose(arm="LEFT", hand="UP")
    car.arm.set_arm_pose(x=-0.2, y=-0.05)
    captures = []  # 后台采集线程按序写入 (det, 裁剪帧)
    alive = [True]  # 采集线程存活开关(车停后置 False, 线程立即退出)
    th = threading.Thread(target=_collect_loop, args=(car, captures, alive), daemon=True)
    th.start()

    def end():
        return len(captures) >= MAX_ANIMALS  # 采集满 4 只即停, 不用等大模型

    car.move_base([CRUISE_SPEED, 0, 0], end)  # 纯直线经行(开环, 不看车道), 采集满 4 只即停

    alive[0] = False
    th.join(timeout=2.0)

    # 停车后统一推理(慢但车已停); 后台线程 + join 超时, 大模型卡死也不堵流程
    results = []
    infer = threading.Thread(
        target=_infer_all, args=(car, captures, results), daemon=True
    )
    infer.start()
    infer.join(timeout=15.0)

    animal_list = list(results) + [1] * (MAX_ANIMALS - len(results))
    print("animal_list =", animal_list)
    return animal_list
