import threading
import time

# 巡线巡航速度(m/s): 纯直线经行(开环, 不看车道, 无转向校正), 恒速扫过 4 只 animal
CRUISE_SPEED = 0.20

# 动物识别置信度阈值: 置信度大于此值的 animal 才送大模型
ANIMAL_CONF = 0.85
MAX_ANIMALS = 4  # 需识别动物数
# 触发裁剪的画面中央窗口(|x_c|<=该值): 目标经过画面中央时裁剪最完整, 避免切边缘残框
CAPTURE_WINDOW = 0.12
MAX_RUN_M = 1.0  # 兜底经行距离(米): 漏检/大模型失败时也停, 防过站
MAX_TIME_S = 6.0  # 兜底总时长(秒)


def _analyze_loop(car, results):
    """后台线程: 巡线经行中监测侧面 animal, 出现即裁剪送大模型, 结果按序入 results.

    状态机: 中央窗口内有 animal 且未捕获当前只 -> 捕获并送大模型(阻塞网络调用
    在此线程内, 不挡巡线); 该只离开中央窗口(或画面)后才预备捕获下一只。
    同一只动物在窗口内停留期间只捕获一次(armed 置 False)。
    """
    armed = True
    while not getattr(car, "_stop_flag", False) and len(results) < MAX_ANIMALS:
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
            res = car.animal_image_analysis(det=a, image=frame)
            results.append(0 if res is None else res)
            print(
                f"[识别] 第 {len(results)}/{MAX_ANIMALS} 只: "
                + ("害" if res == 0 else "益" if res == 1 else f"{res}")
            )
            car.beep()
            armed = False
        elif not armed and abs(a[4]) > CAPTURE_WINDOW:
            armed = True
        time.sleep(0.02)


def run(car) -> list:
    # 每元素 害/益: 害=0 需击打, 益=1
    # 机械臂位姿一次调好, 全程保持(不再为识别逐只停下来)
    car.arm.set_arm_pose(arm="LEFT", hand="UP")
    car.arm.set_arm_pose(x=-0.2, y=-0.05)
    car.get_distance(True)

    results = []  # 后台线程按序写入 害/益
    th = threading.Thread(target=_analyze_loop, args=(car, results), daemon=True)
    th.start()

    start = time.time()

    def end():
        if len(results) >= MAX_ANIMALS:
            return True
        if time.time() - start > MAX_TIME_S:
            return True
        return car.get_distance() > MAX_RUN_M

    car.move_base([CRUISE_SPEED, 0, 0], end)  # 纯直线经行(开环, 不看车道), 识别完/到距/超时即停

    th.join(timeout=2.0)
    animal_list = list(results) + [1] * (MAX_ANIMALS - len(results))
    print("animal_list =", animal_list)
    car.get_distance(True)
    return animal_list
