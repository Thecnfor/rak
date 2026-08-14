# tasks/tools 底层 API 速查（MyCar 运动 / 感知 / 车辆）

本文档覆盖任务真正调用的底层能力：`MyCar` 门面、运动原语、感知接口、机械臂控制、底盘驱动，以及拆分后新增的**串口异步事件机制（并发控制姿态）**。
适用对象：写/改比赛任务的开发者。

---

## 0. 总览：MyCar 是什么

`MyCar(MotionMixin, PerceptionMixin, MecanumDriver)` 定义在 [car.py](file:///home/xrak/Desktop/baidu_smartcar_2026/tasks/tools/car.py)，
继承了三组能力：

| 来源 | 提供 |
|---|---|
| `MecanumDriver`（smartcar 底层） | 底盘运动学 / 里程计 / 速度 / 位姿控制 |
| `MotionMixin`（motion/） | 移动原语：`move_base / move_time / move_distance` |
| | 巡线：`lane_base / lane_time / lane_dis / lane_dis_offset` |
| | 定位：`lane_det_location / move_to_detection_target / det2pose` |
| `PerceptionMixin`（perception/） | 推理初始化、检测查询、实时流、文心分析 |

### 0.1 创建车

```python
from tasks.tools import create_car
car = create_car(reset=True)   # reset=True: 机械臂归位 + 里程计清零
# 用 --no-reset 跳过复位（等价 create_car(reset=False)）
```

`create_car` 内会执行：`car.beep()` → `car.arm.reset_position()` → `car.reset_position()`。

### 0.2 生命周期

```python
car.stop()    # 底盘速度清零
car.close()   # 关按键线程 / 摄像头 / 流媒体（写独立脚本务必调用）
```

### 0.3 MyCar 自带子对象

| 属性 | 类型 | 说明 |
|---|---|---|
| `car.arm` | `ArmController` | 机械臂（见 §3） |
| `car.key` | `Key4Btn` | 车体 4 键 |
| `car.servo_1` | `ServoPwm` | 储存仓舵机（`set_storage` 用） |
| `car.shoot` | `PoutD` | 射击气阀 |
| `car.ring` | `Beep` | 蜂鸣器 |
| `car.blue_pad` | `BluetoothPad` | 蓝牙手柄 |
| `car.display` | `ScreenShow` | 屏幕显示 |
| `car.streamer` | `Streamer` | MJPEG 双路推流 |
| `car.cap_front` / `car.cap_side` | `Camera` | 前视 / 侧视摄像头 |
| `car.crusie` | `ClintInterface("lane")` | 巡线推理客户端（前置） |
| `car.task_det` | `ClintInterface("task")` | 任务检测客户端（侧视） |
| `car.lane_pid` / `car.det_pid` | `PidCal2` | 巡线 / 定位 PID |
| `car.image_analysis` / `car.order_analysis` | `ErnieBotWrap` | 文心图像 / 订单分析 |
| `car._stop_flag` | `bool` | 急停标志（按键 3 置位） |

---

## 1. 底盘驱动（继承自 MecanumDriver）

### 1.1 速度 / 停

| 方法 | 说明 |
|---|---|
| `car.set_velocity(x, y, z)` | 设置底盘速度（x 前后 / y 左右 / z 角速度，单位 m/s 与 rad/s） |
| `car.set_velocity_for_duration(x, y, z, dur)` | 以给定速度运动 dur 秒后自停 |
| `car.stop()` | 速度清零 |

### 1.2 里程计 / 位姿

| 方法 / 属性 | 说明 |
|---|---|
| `car.get_odometry(show_info=False)` | 返回 `[x, y, theta]`（世界坐标，m / rad） |
| `car.get_distance(show_info=False)` | 累计行驶路程（m） |
| `car.reset_position(x=0, y=0, z=0.0, distance=0)` | 重置位姿（默认回原点） |
| `car.x` | 当前 x 位置（mm） |
| `car.move_x(mm)` / `car.move_y(mm)` / `car.move_z(deg)` | 相对移动 / 旋转 |
| `car.offset.x += 100` / `car.offset.y -= 50` / `car.offset.z += 90` | 相对偏移（x/y 为 mm，z 为度） |

### 1.3 坐标变换

| 方法 | 说明 |
|---|---|
| `car.world_to_car_velocity(vel_world, angle_car)` | 世界系速度 → 车体系 |
| `car.car_to_world_velocity(vel_car, angle_car)` | 车体系速度 → 世界系 |

### 1.4 PID 位姿控制

| 方法 | 说明 |
|---|---|
| `car.move_to_position(target, duration=None, max_velocities=(0.2,0.2,π/3), tolerance=(0.004,0.004,0.02), timeout=30.0)` | PID 移动到绝对位姿 `[x,y,theta]`；连续 20 次到位或超时结束 |
| `car.move_for(position_offset, duration, max_velocities, tolerance)` | 基于当前位置叠加相对偏移（m/rad） |
| `car.offset_by(position_offset, duration, max_velocities, tolerance)` | 相对偏移（x/y 为 mm，z 为度） |

> 任务里 `car.move_to_position([x, y, z])` 是"走到某个场上坐标"最常用的接口。

---

## 2. 运动原语（MotionMixin）

### 2.1 基础移动（MoveMixin）

| 方法 | 说明 |
|---|---|
| `car.move_base(sp, end_fuction, stop=True)` | 以速度 `[x,y,z]` 持续移动直到 `end_fuction()` 返回 True |
| `car.move_time(sp, dur_time=1, stop=True)` | 按时间移动 |
| `car.move_distance(sp, dis=0.1, stop=True)` | 按距离移动（基于累计路程差） |
| `car.calculation_dis(pos_dst, pos_src)` | 两点欧氏距离 |

### 2.2 巡线 / 车道保持（LaneMixin）

前视摄像头闭环；`error` 为中线误差、`angle` 为转弯误差；误差大自动降速。

| 方法 | 说明 |
|---|---|
| `car.lane_base(speed, end_fuction, stop=True)` | 巡线直到 `end_fuction()` 返回 True |
| `car.lane_time(speed, time_dur, stop=True)` | 巡线 time_dur 秒 |
| `car.lane_dis(speed, dis_end, stop=True)` | 巡线直到累计路程 > dis_end |
| `car.lane_dis_offset(speed, dis_hold, stop=True)` | 巡线前进 dis_hold 距离 |

### 2.3 目标定位 / 逼近（LocateMixin）

| 方法 | 说明 |
|---|---|
| `car.lane_det_location(speed, pts_tar, dis_out=0.05, side=1, time_out=2, det="task")` | 侧视检测并 PID 定位到目标；`pts_tar` 元素为 `[id, 宽度m, 标签, 置信度, x_c, y_c, w, h]`；返回目标索引或 False |
| `car.move_to_detection_target(delta_x=0.0, delta_y=0.0, label=None, time_out=2.0, sort_pos=(0,0), num=0)` | 视觉对齐目标（配合机械臂横向跟随）；返回 `(cls_id, label)` |
| `car.adjust_arm_position(dis=0.05)` | 按机械臂方向微调横向位置 |
| `car.det2pose(det, w_r=0.06)` | 归一化 bbox + 实际宽度 → 真实坐标 `(x, y, 距离)` |

---

## 3. 机械臂控制（car.arm，ArmController）

机械臂为 **4 自由度（4DoF）**：

- **水平轴 X**（`motor_x`，`MotorWrap`，直线滑轨）
- **竖直轴 Y**（`motor_y`，`StepperWrap`，步进丝杠）
- **手臂舵机**（`arm_servo`，`ServoBus`，总线舵机，方向 LEFT/MID/RIGHT）
- **手部舵机**（`hand_servo`，`ServoPwm`，UP/MID/DOWN）
- **气泵**（`pump`）+ **气阀**（`valve`），即抓取机构

### 3.1 位姿 / 姿态接口

| 方法 / 属性 | 说明 |
|---|---|
| `car.arm.x` / `car.arm.y` | 当前水平 / 竖直位置（mm） |
| `car.arm.angle` / `car.arm.hand_angle` | 手臂 / 手部舵机当前角度 |
| `car.arm.side` | 机械臂方向 `"LEFT" / "MID" / "RIGHT"` |
| `car.arm.arm_length` | 机械臂长度（标定值） |
| `car.arm.x_get_position()` / `car.arm.y_get_position()` | 水平 / 竖直位置（m） |
| `car.arm.x_pose_now` / `car.arm.y_pose_now` | 当前 X / Y 位姿（m） |

### 3.2 同步运动（阻塞，一次完成）

| 方法 | 说明 |
|---|---|
| `car.arm.move_x_position(target, out_time=6.0)` | 水平移到 target（m），PID 闭环，到位/超时/堵转停止 |
| `car.arm.move_y_position(target)` | 竖直移到 target（m），到位/堵转停止 |
| `car.arm.goto_position(x=None, y=None, time_run=None, speed=[0.15, 0.04])` | **双轴并发**移动（见 §6），x/y 可只传一个 |
| `car.arm.go_for(x_offset, y_offset, time_run=None, speed=[0.15, 0.04])` | 相对当前位置偏移双轴 |
| `car.arm.set_arm_angle(angle, speed=80)` | 设置手臂角度（`"LEFT"/"MID"/"RIGHT"` 或数字） |
| `car.arm.set_hand_angle(angle, speed=80)` | 设置手部角度（`"UP"/"MID"/"DOWN"` 或数字） |
| `car.arm.set_arm_pose(x=None, y=None, arm=None, hand=None)` | 组合设置：先 goto_position(x,y)，再设 arm/hand |
| `car.arm.grasp(value: bool)` | `True` 吸起 / `False` 释放（气泵+气阀配合） |
| `car.arm.switch_side(side)` | 切换机械臂方向（带 0.5s 等待） |
| `car.arm.reset_position()` | 复位（Y/X 轴并发复位线程 + 手/臂舵机回正） |
| `car.arm.set_manually()` | 用 4 键手动控制机械臂 |

### 3.3 异步 tick 运动（非阻塞，不独占总线）

| 方法 | 说明 |
|---|---|
| `car.arm.goto_position_async(x=None, y=None, time_run=None, speed=[0.15,0.04], tick_interval=0.02)` | 非阻塞双轴移动：每次调用驱动一 tick，全部到位返回 `True` |
| `car.arm.tick_x_moveto(target)` / `car.arm.tick_y_moveto(target)` | 单轴单步驱动，到位返回 `True` |
| `car.arm.cancel_async_move()` | 取消进行中的异步移动，停止双轴 |
| `car.arm.x_speed_async(velocity)` / `car.arm.y_speed_async(velocity)` | 异步设速（发命令不等应答） |

---

## 4. 感知（PerceptionMixin）

### 4.1 推理客户端（ClintInterface）

```python
car.crusie   # "lane" 模型：巡线
car.task_det # "task" 模型：侧视任务检测
```

- 通过 ZMQ 连接后台推理进程（端口 5001 lane / 5002 task，见 `config_car.yml`）。
- 调用即推理：`res = car.task_det(img)`。
- 检测结果行格式：`[cls_id, obj_id, label, score, x_c, y_c, w, h]`，坐标**归一化**（约 -1~1）。

### 4.2 检测查询（DetectMixin）

| 方法 | 说明 |
|---|---|
| `car.get_detection_results(sort_pos=(0,0), limit_x=1, limit_y=1)` | 侧视读帧→task 推理→按离 sort_pos 近远排序→返回检测列表；同时写入实时缓存 |
| `car.get_lane_results()` | 巡线推理，返回滤波后的 `(error, angle)`；异常保持上一帧，超时按 0 直行 |
| `car.get_target_location(det)` | 由检测框计算目标相对小车的 `(loc_x, loc_y)` |
| `car.draw_detection_results(img, dets_ret)` | 在图上画检测框并返回 |

### 4.3 实时流与持续检测（RealtimeMixin）

`MyCar` 初始化时自动启动侧视（cam2）实时检测 + 推流线程。

| 方法 | 说明 |
|---|---|
| `car.get_realtime_detections(fresh=False, max_age=None)` | 非阻塞拿最新侧视检测结果；`fresh=True` 立刻同步跑一次推理 |
| `car.get_realtime_side_frame(with_overlay=True)` | 拿最新侧视画面（可带检测框叠加） |
| `car.start_side_stream()` | 手动启动侧视流线程（一般已自动启动） |

### 4.4 文心分析（OcrErnieMixin / InferInitMixin）

| 方法 | 说明 |
|---|---|
| `car.animal_image_analysis()` | 裁剪检测到的目标图 → 文心识别动物 → `(result, analysis)`，result 为 0(有害)/1(有益) |
| `car.yiyan_get_humattr(text)` | 人属性分析（需配置 hum_analysis） |
| `car.yiyan_get_actions(text)` | 动作分析（需配置 action_bot） |
| `car.get_ocr(label=None, time_out=3.0)` / `car.get_det_ocr(det, label, time_out=5.0)` | OCR 识别（当前 `ocr_rec=None` 已停用，会直接返回 None） |

> `image_analysis.get_image_res(base64)` / `order_analysis.get_res_json(text)` 可单独调用（见 §7）。

### 4.5 MyCar 内建动作

| 方法 | 说明 |
|---|---|
| `car.beep()` | 蜂鸣一声 |
| `car.set_storage(state)` | 储存仓开合（`False` 放下 / `True` 收起） |
| `car.shooting()` | 射击机构触发（气阀 0.3s） |
| `car.delay(time_hold)` | 延时（期间检查急停标志，可被打断） |

---

## 5. 摄像头 / 推流 / 日志 / 工具

### 5.1 Camera

```python
from smartcar import Camera
cap = Camera(index=3, width=640, height=480)   # index 对应 /dev/cam3（udev 符号链接）
img = cap.read()          # 阻塞取最新帧
cap.set_size(w, h)        # 改分辨率
cap.close()
```

### 5.2 Streamer（MJPEG 双路推流）

```python
from smartcar import Streamer
s = Streamer(port=5000, fps=30)
s.update_frame(img, "cam1")   # 推送某路画面
s.get_key(clear=True)         # 取网页按键
s.stop()
```

### 5.3 日志

```python
from smartcar import logger
logger.debug/info/warning/error/critical(...)
# 写 logs/ 目录，按日滚动，保留 10 天
```

### 5.4 通用工具（smartcar 顶层导出）

| 工具 | 说明 |
|---|---|
| `PID(Kp, Ki, Kd, setpoint, output_limits, ...)` | 增量式 PID（simple_pid 风格） |
| `PidCal2(cfg_pid_y, cfg_pid_angle)` | 双 PID 集合：`get_out(error_y, error_angle) -> (out_y, out_angle)` |
| `CountRecord(stop_count)` | 计数滤波：连续 N 次相同值才返回 True |
| `IndexWrap(num, circle=False)` | 循环索引器：`next/before/get_index` |
| `limit_val(val, min_val, max_val)` | 限幅 |
| `get_yaml(path)` | 读 yaml 配置 |

---

## 6. 并发控制机械臂姿态（重点）

### 6.1 物理前提：单 MC602 串口

底盘/机械臂/按键/舵机/气泵**共享同一个串口**（`serial_wrap` 单例，一个 fd）。
物理上串口一问一答，**任何时刻总线只能串行传一条命令**。所以"并发"不是同时发包，而是：
**读线程常驻后台解析应答，发命令不等应答立即返回 → 调用方交错发不同设备的命令**。

### 6.2 异步串口引擎（拆分后：serial_engine.AsyncSerialEngine）

- 常驻 daemon 读线程 + `select` + 帧状态机，自动切帧并分发。
- 发送 `submit(cmd)` 立即返回，登记 pending；应答到达时按 FIFO 唤醒同步等待者 / 触发回调。
- **兼容层不变**：`serial_wrap.get_anwser(cmd, time_out)` 仍是"提交+等应答"的同步语义，所有旧代码零改动。
- 回退开关：`SMARTCAR_SERIAL_SYNC=1` 环境变量一键切回旧锁式一问一答。

### 6.3 机械臂双轴"并发"的实际做法

机械臂 X / Y 是两个独立电机，**可以同时发速度命令**（两个 PID 各自闭环）。

**① 同步并发（推荐，简单）** —— `goto_position` 内部双轴 PID 交替发速度，同一循环里同时推进 X、Y：

```python
# 同时移动 X 到 0.30、Y 到 0.20，双轴并发（内部交替 setpoint，直到都到位）
car.arm.goto_position(x=0.30, y=0.20)

# 只动一个轴
car.arm.goto_position(x=0.30)          # y=None 表示 Y 不动
car.arm.goto_position(y=0.20)          # x=None 表示 X 不动

# 相对偏移
car.arm.go_for(x_offset=0.05, y_offset=-0.03)
```

**② 异步非阻塞（配合事件循环，不独占总线）** —— 在任务主循环里每个 tick 调一次，
期间还能穿插底盘/感知命令（因为速度命令是异步发的，不等应答）：

```python
# 不阻塞主循环的双轴移动：每 tick 双轴各发一条异步速度命令
while not car.arm.goto_position_async(x=0.30, y=0.20):
    car.delay(0.02)          # 期间可做别的事（读检测、走底盘等）

# 取消
car.arm.cancel_async_move()
```

**③ 单轴异步设速**（最细粒度，完全自定义轨迹）：

```python
car.arm.x_speed_async(0.1)   # X 以 0.1 m/s 异步前进
car.arm.y_speed_async(-0.05) # 同时 Y 异步下降（双轴并行）
```

### 6.4 复位已经是并发的

`car.arm.reset_position()` 用两个 daemon 线程分别跑 `reset_x` / `reset_y` 并发归位，然后 `join`。

### 6.5 与其他功能的并发（重要结论）

- **机械臂运动 + 底盘运动**：可以。底盘的 `move_to_position` 等走 PID 速度命令（每次 `set_velocity` 是一发），机械臂的异步速度命令与之交错，读线程保证各自应答不串包。
- **机械臂运动 + 按键 / 蜂鸣 / 舵机**：可以。异步引擎按 `(dev_id, mode, port)` 订阅分发，互不干扰。
- **不要**在机械臂同步运动（`move_x_position` / `goto_position` 阻塞版）内部再从别的线程发同设备命令——那是同一个电机，两个闭环会打架。
- **固件下载 / 独占阶段**：`serial_wrap.pause_rx()` / `resume_rx()` 会暂停读线程，其它命令在此期间不应发出。

### 6.6 事件驱动（回调）写法

设备对象支持订阅自己的应答/上报帧：

```python
# 订阅侧视模拟量传感器的上报帧（收到即回调）
car.arm.y_limit_sensor.subscribe(lambda res: logger.info(f"limit raw: {res}"))
# 解除
car.arm.y_limit_sensor.unsubscribe(lambda res: logger.info(...))

# 电机异步设速 + 回调
motor2.set_speed_async(50, callback=lambda res: logger.info(f"ack: {res}"))
```

底层接口（一般用不上，直接走设备对象即可）：

- `serial_wrap.subscribe(dev_id, mode, port, callback)` / `unsubscribe(...)`
- `serial_wrap.send_async(cmd, callback=None, timeout=0.2)`
- `serial_wrap.send_raw(data)`（不带帧头尾的直写）
- `serial_wrap.pause_rx()` / `resume_rx()`

---

## 7. 文心 API（ErnieBotWrap）

```python
from smartcar import ErnieBotWrap, ImagePrompt, OrderPrompt

bot = ErnieBotWrap()
bot.set_promt(str(OrderPrompt()))          # 设置 schema
res = bot.get_res_json("2号楼李四要芹菜")    # 返回解析后的 dict
# {"name": "李四", "goods": "芹菜", "address": 2}

# 图片分析（返回 (result, analysis)，result 0 有害 / 1 有益）
result, analysis = car.image_analysis.get_image_res(base64_image_str)
```

> `config_car.yml` 的 `ernie_access_token` 是文心密钥（占位符，部署时填真值）。

---

## 8. 常用组合套路（示例）

### 8.1 巡线 + 定点 + 视觉对齐 + 抓取

```python
car.arm.set_arm_pose(0.0, 0.2, "LEFT", "DOWN")   # 起始姿态
car.lane_dis_offset(speed=0.3, dis_hold=0.85)     # 巡线前进
car.move_to_position([0.45, 0.35, 0.78])          # 到播种点
car.move_to_detection_target()                    # 视觉对齐
car.arm.grasp(True)                               # 吸起
car.arm.move_y_position(0.2)                      # 抬臂
car.arm.grasp(False)                              # 放下
```

### 8.2 双轴并发 + 底盘并发（异步）

```python
# 机械臂异步双轴移动的同时，底盘走位姿
while not car.arm.goto_position_async(x=0.30, y=0.20):
    car.delay(0.02)
car.move_to_position([0.6, 0.4, 0])
```

---

## 9. 关键配置文件

| 文件 | 内容 |
|---|---|
| `config_car.yml` | 摄像头通道（front:3 / side:4）、IO、PID、ZMQ 端口（5001/5002）、模型目录、文心 token |
| `smartcar/whalesbot/vehicle/arm/arm_cfg.yaml` | 机械臂标定：`arm_length` / `vert_cfg` / `horiz_cfg` / `hand_cfg` / `pos_cfg` |
| `smartcar/whalesbot/vehicle/driver/cfg_vehicle.yaml` | 底盘：轮距/轴距/轮径/电机端口/PID |
| `smartcar/paddlebaidu/models/task2026/labels.txt` | task 模型 23 类标签（改标签需同步改任务逻辑） |

---

## 10. 常见坑

- **检测坐标是归一化** `[x_c, y_c, w, h]`（约 -1~1），画框/裁剪要先转像素。
- `move_to_detection_target` 会同时驱动底盘横移和 `car.arm.x_speed`，别和 `move_x_position` 一起用。
- OCR 已停用（`ocr_rec = None`），`get_ocr` 会直接返回 None；识别名称/订单走文心。
- 修改机械臂标定常量会改变场上物理行为，除非重新标定，否则不要动。
