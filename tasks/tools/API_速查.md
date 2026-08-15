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
| `car.set_velocity(x, y, z)` | 设置底盘速度（x 前后 / y 左右 / z 角速度，单位 m/s 与 rad/s）；**开环**，无纠偏 |
| `car.set_velocity_for_duration(x, y, z, dur)` | 以给定速度运动 dur 秒后自停 |
| `car.stop()` | 速度清零 |

### 1.2 里程计 / 位姿

| 方法 / 属性 | 说明 |
|---|---|
| `car.get_odometry(show_info=False)` | 返回 `[x, y, theta]`（世界坐标，m / rad） |
| `car.get_distance(show_info=False)` | 累计行驶路程（m）；注意其按 `dx,dy 模长之和` 累计，**转弯/横移也会计入** |
| `car.reset_position(x=0, y=0, z=0.0, distance=0)` | 重置位姿（默认回原点） |
| `car.x` | 当前 x 位置（mm） |
| `car.move_x(mm)` / `car.move_y(mm)` / `car.move_z(deg)` | 相对移动 / 旋转 |
| `car.offset.x += 100` / `car.offset.y -= 50` / `car.offset.z += 90` | 相对偏移（x/y 为 mm，z 为度） |

### 1.3 坐标变换

| 方法 | 说明 |
|---|---|
| `car.world_to_car_velocity(vel_world, angle_car)` | 世界系速度 → 车体系 |
| `car.car_to_world_velocity(vel_car, angle_car)` | 车体系速度 → 世界系 |

### 1.4 PID 位姿控制（闭环，推荐直线 / 定点）

> `move_to_position / move_for / offset_by` 是 **带 x/y/yaw 三路 PID 的闭环**控制（基于里程计），
> 相比 `set_velocity` 开环直走能自动修正跑偏，是"直行 / 到点"的最稳方式。

| 方法 | 说明 |
|---|---|
| `car.move_to_position(target, duration=None, max_velocities=(0.2,0.2,π/3), tolerance=(0.004,0.004,0.02), timeout=30.0)` | PID 闭环移动到绝对位姿 `[x,y,theta]`（m/rad）；连续 20 次到位或超时/千次迭代结束 |
| `car.move_for(position_offset, duration, max_velocities, tolerance)` | 基于当前位置叠加相对偏移（m/rad）；按当前朝向做平移坐标变换 |
| `car.offset_by(position_offset, duration, max_velocities, tolerance)` | 相对偏移（x/y 为 mm，z 为度），内部转调 `move_for` |

> 任务里 `car.move_to_position([x, y, z])` 是"走到某个场上坐标"最常用的接口。
> `move_for([x, y, 0])` / `offset_by([x_mm, y_mm, z_deg])` 则适合"相对当前位置直走 / 横移 / 转向"。

### 1.5 底盘内部对象（一般无需直接操作）

`car.chassis`（MecanumChassis）与 `car.chassis.odometry`（Odometry）提供运动学与位姿数据：

| 对象 / 方法 | 说明 |
|---|---|
| `car.chassis.odometry.position` | 位姿数组 `[x, y, theta]`（m/rad），**直接字段，不走锁**，只读场景可用 |
| `car.chassis.odometry.distance` | 累计行驶路程（m） |
| `car.chassis.odometry.velocity` | 速度数组（世界系） |
| `car.chassis.calculate_wheel_velocities(x, y, z)` | 逆解：车速度 → 4 轮线速度 |
| `car.chassis.forward_kinematics(wheel_vel)` / `inverse_kinematics(car_vel)` | 运动学正 / 逆解 |
| `car.chassis.update_odometry(disp)` | 按轮子位移更新里程计（由后台线程调用） |
| `car.wheels_chassis` | `WheelWrap`，`set_linear / get_linear / get_linear_async` 直接控制/读 4 轮 |
| `car.pid_x / car.pid_y / car.pid_yaw` | 底盘三路 PID（`move_to_position` 使用） |

> 里程计由后台 `update_odometry_thread` 以 ~50Hz 异步更新（`get_linear_async` 读编码器差分），
> 通过 `car._lock` 保护；`get_odometry/get_distance` 已带锁，任务层直接用即可。

### 1.6 生命周期

| 方法 | 说明 |
|---|---|
| `car.stop()` | 底盘速度清零 |
| `car.close()` | 停里程计线程（`_stop_thread=True` + join） |

---

### 1.7 补充工具（smartcar 顶层已导出，可直接 import）

`MyCar` 已继承了底层全部能力，任务一般无需直接 import 下列工具；它们与 `tasks/tools` 里的自定义封装可互相替代，统一从 `smartcar` 导入即可：

| 工具 | 来源 | 说明 / 可替代的 tools 内实现 |
|---|---|---|
| `from smartcar import PID` | `tools/tools_class.py` | 增量式 PID；`PidCal2` 内部就是两个 PID |
| `from smartcar import get_yaml` | `tools/tools_class.py` | 读 yaml，失败返回 None |
| `from smartcar import CountRecord` | `tools/tools_class.py` | 计数滤波（连续 N 次相同才 True） |
| `from smartcar import IndexWrap` | `tools/tools_class.py` | 循环索引器 |
| `from smartcar import limit_val` | `tools/tools_class.py` | 限幅（`whalesbot.tools.limit_val`） |
| `from smartcar import logger` | `tools/log_wrap.py` | 滚动日志 |
| `from smartcar.whalesbot.vehicle import ArmController, MecanumDriver` | driver / arm | 机械臂 / 底盘主类 |

> `smartcar/__init__.py` 已统一 re-export 上表工具，`tasks/tools` 内重复封装时优先引用 smartcar 版本，避免双实现漂移。

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

> `ArmController(ArmMotion)`：X/Y 双轴运动能力来自 `arm_motion.py` 的 `ArmMotion` mixin，
> 手部 / 姿态 / 复位 / 属性接口在 `arm_base.py`。两者组合成完整机械臂 API。

### 3.1 位姿 / 姿态接口

| 方法 / 属性 | 说明 |
|---|---|
| `car.arm.x` / `car.arm.y` | 当前水平 / 竖直位置（mm）；**可写**：赋值即移动（内部转调 move_x/y_position） |
| `car.arm.angle` / `car.arm.hand_angle` | 手臂 / 手部舵机当前角度；**可写** |
| `car.arm.side` | 机械臂方向 `"LEFT" / "MID" / "RIGHT"` |
| `car.arm.arm_length` | 机械臂长度（标定值） |
| `car.arm.x_get_position()` / `car.arm.y_get_position()` | 水平 / 竖直位置（m） |
| `car.arm.x_pose_now` / `car.arm.y_pose_now` | 当前 X / Y 位姿（m） |
| `car.arm.x_pose_start` / `car.arm.y_pose_start` | X / Y 零点基准（m） |
| `car.arm.x_threshold` / `car.arm.y_threshold` | X / Y 移动限位 `[min, max]`（m） |
| `car.arm.x_velocity_limit` / `car.arm.y_velocity_limit` | X / Y 速度限幅（PID output_limits） |
| `car.arm.motor_x` / `car.arm.motor_y` | X（MotorWrap）/ Y（StepperWrap）电机对象 |
| `car.arm.y_limit_sensor` | Y 轴限位传感器（`AnalogInput`，`read()>1000` 视为到限位） |

### 3.2 同步运动（阻塞，一次完成）

| 方法 | 说明 |
|---|---|
| `car.arm.move_x_position(target, out_time=6.0)` | 水平移到 target（m），PID 闭环，到位/超时/堵转停止 |
| `car.arm.move_y_position(target)` | 竖直移到 target（m），到位/堵转停止 |
| `car.arm.goto_position(x=None, y=None, time_run=None, speed=[0.15, 0.04])` | **双轴并发**移动（见 §6），x/y 可只传一个；`time_run` 给运行秒数或 `speed` 自动算时长 |
| `car.arm.go_for(x_offset, y_offset, time_run=None, speed=[0.15, 0.04])` | 相对当前位置偏移双轴 |
| `car.arm.set_arm_angle(angle, speed=80)` | 设置手臂角度（`"LEFT"/"MID"/"RIGHT"` 或数字） |
| `car.arm.set_hand_angle(angle, speed=80)` | 设置手部角度（`"UP"/"MID"/"DOWN"` 或数字） |
| `car.arm.set_arm_pose(x=None, y=None, arm=None, hand=None)` | 组合设置（同步）：先 goto_position(x,y) 阻塞，再设 arm/hand |
| `car.arm.grasp(value: bool)` | `True` 吸起 / `False` 释放（气泵+气阀配合） |
| `car.arm.switch_side(side)` | 切换机械臂方向（带 0.5s 等待） |
| `car.arm.reset_position()` | 复位（Y/X 轴并发复位线程 + 手/臂舵机回正） |
| `car.arm.set_manually()` | 用 4 键手动控制机械臂 |
| `car.arm.x_speed(velocity)` / `car.arm.y_speed(velocity)` | 直接设 X / Y 轴速度（内部限幅） |
| `car.arm.set_position_start(y_position)` | 把当前 X/Y 位置记录为零点基准并保存配置 |

### 3.3 异步 tick 运动（非阻塞，不独占总线）

| 方法 | 说明 |
|---|---|
| `car.arm.goto_position_async(x=None, y=None, time_run=None, speed=[0.15,0.04], tick_interval=0.02)` | 非阻塞双轴移动：每次调用驱动一 tick，全部到位返回 `True` |
| `car.arm.tick_x_moveto(target)` / `car.arm.tick_y_moveto(target)` | 单轴单步驱动，到位返回 `True` |
| `car.arm.cancel_async_move()` | 取消进行中的异步移动，停止双轴 |
| `car.arm.x_speed_async(velocity)` / `car.arm.y_speed_async(velocity)` | 异步设速（发命令不等应答） |

### 3.4 四轴并发（X/Y 双轴 + 手臂/手部舵机并行）

机械臂 4DoF 现已支持**四自由度并发**：X/Y 走异步 PID，手臂/手部舵机角度命令异步发出，四个自由度**并行开始、并行结束**，总耗时 ≈ max(各轴) 而非 sum。

| 方法 | 说明 |
|---|---|
| `car.arm.set_arm_angle_async(angle, speed=80, callback=None)` | 异步设置手臂角度（`"LEFT"/"MID"/"RIGHT"` 或数字），立即返回 |
| `car.arm.set_hand_angle_async(angle, speed=80, callback=None)` | 异步设置手部角度（`"UP"/"MID"/"DOWN"` 或数字），立即返回 |
| `car.arm.set_arm_pose_async(x=None, y=None, arm=None, hand=None)` | 四轴并发组合：arm/hand 舵机异步发出 + XY 异步移动驱动一个 tick |

```python
# 四轴并发: 一条命令让 X/Y + 手臂 + 手部全部并行开始
car.arm.set_arm_pose_async(x, y, arm="RIGHT", hand="DOWN")

# 主循环里每 tick 驱动 XY, 直到双轴到位(舵机角度命令已先行异步发出)
while not car.arm.goto_position_async(x, y):
    car.delay(0.02)          # 期间可做别的事
```

> 同步版 `set_arm_angle` / `set_hand_angle` / `set_arm_pose` 保持不变，旧代码不受影响。

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
| `car.get_lane_results()` | 源头合成后的新一对 `(steer, da)`：steer 为 correction 模型、da 为 lane 模型；异常保持上一帧，超时按 0 直行 |
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
- **异步项超时清理**：读线程每 1s 清理超时未回包的异步项（触发回调 `None`），防止 MC602 不应答时僵尸项卡住后续 FIFO。
- 回退开关：`SMARTCAR_SERIAL_SYNC=1` 环境变量一键切回旧锁式一问一答。

### 6.2b 异步高频读（里程计 / 按键，不阻塞总线）

后台高频读取（编码器/按键）已改为**异步发命令 + 回调更新**，主循环只 sleep，不占总线：

| 接口 | 说明 |
|---|---|
| `DevListWrap.get_all_async(args, mode, callback, timeout)` | 多设备合并一帧异步读，回包回调结果列表 |
| `Motors_2.get_encoder_async(callback, timeout)` | 异步读 4 路编码器，回调编码值列表 |
| `WheelWrap.get_linear_async(callback, timeout)` | 异步读轮子线速度（编码器弧度×半径） |
| `Key4Btn_2.get_key_async(callback, port_id, timeout)` / `Key4Btn.get_key_async(...)` | 异步读当前按键号，回调按键号 |
| `MecanumDriver.update_odometry_thread` | 里程计线程：异步读编码器→回调更新里程计，0.1s 节奏 |
| `MyCar.key_thread_func` | 按键线程：异步读按键→回调置 `_stop_flag`，0.2s 节奏 |

```python
# 里程计/按键都是内部线程, 任务层无需感知; 需要时也可手动用异步读:
car.wheels_chassis.get_linear_async(lambda v: print(v))
car.key.get_key_async(lambda k: print("键:", k))
```

> 超时/失败时回调收到 `None`（而非卡死）。MC601 无异步能力时这些方法自动回落到同步。

### 6.3 机械臂四轴"并发"的实际做法

机械臂 4DoF：X / Y 是两个独立电机（各自 PID 闭环），手臂/手部是两个舵机（设目标角度后自己转动）。
**四轴可以同时发命令**：速度命令（X/Y）+ 角度命令（舵机）异步发出，四轴并行开始/结束。

**① 四轴并发（推荐）** —— `set_arm_pose_async` 把 X/Y 异步移动 + 两条舵机角度命令一次性发出：

```python
# 一条调用: X/Y 双轴异步移动 + 手臂/手部角度异步命令, 四轴并行开始
car.arm.set_arm_pose_async(x=0.30, y=0.20, arm="RIGHT", hand="DOWN")

# 主循环每 tick 驱动 XY, 直到双轴到位(舵机角度命令已先行发出)
while not car.arm.goto_position_async(x=0.30, y=0.20):
    car.delay(0.02)          # 期间可做别的事（读检测、走底盘等）
```

**② 同步并发（简单）** —— `goto_position` 内部双轴 PID 交替发速度，同一循环里同时推进 X、Y：

```python
# 同时移动 X 到 0.30、Y 到 0.20，双轴并发（内部交替 setpoint，直到都到位）
car.arm.goto_position(x=0.30, y=0.20)

# 只动一个轴
car.arm.goto_position(x=0.30)          # y=None 表示 Y 不动
car.arm.goto_position(y=0.20)          # x=None 表示 X 不动

# 相对偏移
car.arm.go_for(x_offset=0.05, y_offset=-0.03)
```

**③ 舵机角度异步单独用**（例如只转手部，不与 XY 联动）：

```python
car.arm.set_arm_angle_async("RIGHT")       # 手臂异步转到 RIGHT
car.arm.set_hand_angle_async("DOWN")       # 手部异步转到 DOWN(与手臂并行)
```

**④ 单轴异步设速**（最细粒度，完全自定义轨迹）：

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
- **`set_velocity` 是开环**：直走会因四轮机械/摩擦差异跑偏；要"稳直行 / 稳到点"用 `move_to_position / move_for / offset_by`（带 x/y/yaw PID 闭环）。
- **`get_distance` 按 `dx,dy 模长之和` 累计**：转弯 / 横移也会计入路程，用它做"直线 2 米"终点判定会被横向位移提前触发，实际前进不足。
- **机械臂位置是相对的**：`x_pose_now/y_pose_now = 电机位移 - x_pose_start/y_pose_start`，零点由 `reset_position` / `set_position_start` 建立；电机 `get_dis()` 是绝对位移，别直接拿来做目标。
- **堵转/超时兜底**：X 轴 `move_x_position(out_time=6.0)`、Y 轴 `move_y_position`、双轴 `goto_position` 都有超时/堵转检测（`STOP_CHECK_THRESHOLD` / `RESET_TIMEOUT`），卡住会自动停，不要自己再开无限循环。
- **异步移动要外部驱动**：`goto_position_async / tick_*` 每次调用只推一 tick，必须放在 `while not ...:` 循环里，否则轴不动。

---

## 11. tasks/tools 与 smartcar 底层可替代对照

`tasks/tools`（MyCar 门面）把 smartcar 底层能力**按比赛用途重新封装**；绝大多数情况下直接调 `car.xxx` 即可，无需 import smartcar。
下表是**可用 smartcar 底层直接替代**的 items/tools 内实现（避免重复造轮子）：

| tools 内实现（文件 / 类） | 可替代它的 smartcar 底层 | 结论 |
|---|---|---|
| `car`（MyCar 门面） | `MecanumDriver` + `MotionMixin` + `PerceptionMixin` | **MyCar 已继承**，无需 import smartcar；直接 `car.xxx` |
| `car.set_velocity / get_odometry / get_distance / reset_position / stop / close` | `smartcar.whalesbot.vehicle.driver.mecanum.MecanumDriver` 同名方法 | **直接继承**，无重复实现 |
| `car.move_to_position / move_for / offset_by / move_x / move_y / move_z / offset.x+=...` | 同左（`MecanumDriver`） | **直接继承** |
| `car.world_to_car_velocity / car_to_world_velocity` | 同左（`MecanumDriver`） | **直接继承** |
| `car.arm.*`（全部机械臂接口） | `smartcar.whalesbot.vehicle.arm.ArmController`（+ `arm_motion.ArmMotion`） | **直接继承**（`MyCar.arm = ArmController()`） |
| `car.get_lane_results` / `car.lane_pid` | `smartcar.paddlebaidu.infer_cs.ClintInterface`（"lane"+"correction" 模型，源头合成新一对 `(steer, da)`）+ `PidCal2` | **tools 封装**，保留 |
| `car.get_detection_results` / `car.task_det` | `ClintInterface`（"task" 模型） | **tools 封装**（含排序/画框/缓存），保留 |
| `car.move_to_detection_target / lane_det_location / det2pose / adjust_arm_position` | 无底层等价 | **tools 独有**（视觉对齐），保留 |
| `PidCal2`（tasks/tools/pids.py） | `from smartcar import PID`（两个 PID 组合） | **可用 smartcar 直接替代**：`PID` 已在 smartcar 导出 |
| `CountRecord / get_yaml / limit_val`（tools 内引用） | `from smartcar import CountRecord, get_yaml` / `whalesbot.tools.limit_val` | **可用 smartcar 替代**（同一定义） |
| `RealtimeMixin`（侧视流/实时检测） | `Streamer` + `ClintInterface` | **tools 封装**，保留 |
| `OcrErnieMixin`（文心/OCR） | `ErnieBotWrap` / `ClintInterface` | **tools 封装**（OCR 已停用），保留 |

> 结论：**凡是"运动 / 里程计 / 机械臂"能力，tools 层只是透传 smartcar（MyCar 继承）**，
> 写任务时直接 `car.xxx` 即可；**不要**在 `tasks/tools` 里再写一份 `set_velocity / move_x_position` 之类同名封装。
> 只有**感知（检测/巡线/视觉对齐/实时流/文心）** 是 tools 层真正的业务封装，smartcar 底层没有对应高能级 API。
