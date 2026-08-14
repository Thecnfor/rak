# 任务层 API 速查（tasks/）

本文档覆盖 `tasks/` 任务层与 `tasks/tools/` 的入口关系，侧重"怎么跑任务、怎么编排、任务函数签名"。
底层硬件/感知 API 见 `tasks/tools/API_速查.md`。

---

## 1. 顶层入口：run.py

所有比赛任务的统一入口（在项目根目录运行）。

```bash
# 完整比赛流程（按键驱动，可中途重来/跳过/急停）
python3 run.py all

# 单任务
python3 run.py seeding              # 播种
python3 run.py target-detection     # 目标识别（返回识别结果并打印）
python3 run.py watering             # 浇水
python3 run.py shooting             # 射击
python3 run.py harvesting           # 收割
python3 run.py sorting              # 分拣
python3 run.py ordering             # 下单
python3 run.py delivery             # 配送

# 跳过开机复位（机械臂归位 + 里程计清零）
python3 run.py seeding --no-reset
```

- 任务内部都用 `car.stop()` / `car.close()` 兜底收尾。
- 任务注册表见 [run.py](file:///home/xrak/Desktop/baidu_smartcar_2026/run.py) 的 `RUNNERS`：任务名 → `tasks.xxx.run`。

### 1.1 任务执行顺序（比赛顺序）

由 `Orchestrator.TASK_ORDER` 定义，与 `run.py` 的 `RUNNERS` 顺序一致：

```
seeding -> target_detection -> watering -> shooting -> harvesting -> sorting -> ordering -> delivery
```

### 1.2 一键启动（按键驱动）

| 按键 | 功能 |
|---|---|
| 4 | 一键启动 / 重来（重来只跑未完成的任务） |
| 1 | 跳过当前任务（不标记完成，下次重来可补做） |
| 3 | 急停（终止本轮；重来从头跑） |

> 已完成任务仅记录在内存，重启程序即全新一次。

---

## 2. 任务模块（tasks/*.py）

每个任务都是独立模块，统一入口 `run(car)`。任务内部通过 `car.xxx` 调用底层 API。

| 模块 | 入口 | 说明 |
|---|---|---|
| [seeding.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/seeding.py) | `run(car)` | 播种：巡线→走到 3 个播种点→吸起圆柱→放下 |
| [target_detection.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/target_detection.py) | `run(car)` | 目标识别（侧视检测 + 文心分析） |
| [watering.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/watering.py) | `run(car)` | 浇水 |
| [shooting.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/shooting.py) | `run(car, animal_list=None)` | 射击（第二参数可选，参数化目标动物序列） |
| [harvesting.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/harvesting.py) | `run(car)` | 收割 |
| [sorting.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/sorting.py) | `run(car)` | 分拣 |
| [ordering.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/ordering.py) | `run(car)` | 下单（返回订单信息） |
| [delivery.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/delivery.py) | `run(car, order_list=None)` | 配送（第二参数可选，参数化配送订单） |

> `shooting.run` / `delivery.run` 的可选第二参数是"序列参数化"的覆盖钩子，`run.py` 目前都用默认值调用。
> 硬改任务里标定的距离/位姿会改变场上物理行为，除非在重新标定，否则不要动。

---

## 3. 任务编排：tasks/orchestrator.py

`Orchestrator` 负责按键采集、任务调度、跳过/急停。

```python
from tasks.orchestrator import Orchestrator

orch = Orchestrator(car)
orch.wait_start()                # 阻塞等按键 4 启动
for task_name in orch.schedule():  # 产出本轮要跑的任务名（跳过已完成）
    orch.start_skip_listener()   # 任务期间监听按键 1（跳过）/ 3（急停）
    try:
        RUNNERS[task_name](car)
    finally:
        skipped, emergency = orch.stop_skip_listener()
    ...
    orch.mark_done(task_name)    # 标记完成（内存内生效）
```

**常量：**

- `TASK_ORDER`：比赛执行顺序列表
- `KEY_START = 4` / `KEY_SKIP = 1` / `KEY_EMERGENCY = 3`

**主要方法：**

| 方法 | 说明 |
|---|---|
| `wait_start()` | 阻塞等待按键 4；按 3 则急停退出 |
| `schedule()` | 返回本轮要执行的任务名列表 |
| `start_skip_listener()` / `stop_skip_listener()` | 任务期间监听跳过/急停，`stop` 返回 `(skipped, emergency)` |
| `mark_done(name)` | 标记某任务完成 |
| `abort()` | 终止按键线程 |

---

## 4. 自定义任务的写法

新任务只需写一个 `run(car)` 函数，参数是 `MyCar` 实例，然后注册到 `RUNNERS`。

```python
def run(car):
    """我的新任务"""
    car.arm.set_arm_pose(0.0, 0.2, "LEFT", "DOWN")   # 机械臂起始姿态
    car.lane_dis_offset(speed=0.3, dis_hold=0.85)     # 巡线走一段
    car.move_to_position([0.45, 0.35, 0.78])          # 到指定点位
    car.move_to_detection_target()                    # 视觉对齐
    car.arm.grasp(True)                               # 吸起
    car.arm.move_y_position(0.2)                      # 抬臂
    car.arm.grasp(False)                              # 放下
    car.beep()
```

> 所有可用的 `car.xxx` 接口见 `tasks/tools/API_速查.md`。

---

## 5. 目录结构速览

```
tasks/
├── __init__.py          # 导出全部任务模块
├── run.py 的映射表       # RUNNERS: 任务名 -> tasks.xxx.run
├── orchestrator.py      # 按键驱动的任务编排器
├── seeding.py           # 播种
├── target_detection.py  # 目标识别
├── watering.py          # 浇水
├── shooting.py          # 射击
├── harvesting.py        # 收割
├── sorting.py           # 分拣
├── ordering.py          # 下单
├── delivery.py          # 配送
└── tools/               # 任务工具层（见 tools/API_速查.md）
    ├── car.py           # MyCar 门面 + create_car()
    ├── pids.py          # PidCal2
    ├── motion/          # MoveMixin / LaneMixin / LocateMixin
    └── perception/      # InferInitMixin / DetectMixin / RealtimeMixin / OcrErnieMixin
```
