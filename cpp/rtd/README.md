# rtd —— Jetson 实时控制守护进程 (C++)

接管 `/dev/ttyUSB0`（MC602 控制器），以固定 100Hz 完成：**读编码器 → 积分里程计 → 速度/位置闭环 PID → 下发轮速**。Python 侧通过 ZMQ 下发命令，并通过同一串口透传机械臂/舵机/按键/蜂鸣器等设备的帧。

所有语义均复刻自仓库 Python 实现（`mc602_devbase.py` / `mecanum.py` / `controller_wrap.py` / `tools_class.py` / `serial_protocol.py`），包括现场标定的怪癖。**rtd 只新建文件，不改动仓库里任何现有文件。**

> 安全提示：`rtd` 会以 `TIOCEXCL` 独占打开串口。**上车前必须确认 Python 进程已停止**；联调一律用 `--simulate`，不要连真实串口。

---

## 1. 架构（文字版）

```
┌─────────────────────────────────────────────────────────────────┐
│ Python 侧 (另行实现)                                              │
│   ZMQ REQ ───── 命令: vel / goto / cancel_goto / stop / reset /  │
│                      state / frame / frame_async                │
│   ZMQ SUB ◄────── 50Hz 状态 + reply / timeout 事件               │
└───────────┬─────────────────────────────▲───────────────────────┘
            │ tcp://127.0.0.1:6010       │ tcp://127.0.0.1:6011
┌───────────▼─────────────────────────────┴───────────────────────┐
│ rtd 进程                                                         │
│   ┌─────────────┐   ┌──────────────┐   ┌──────────────────────┐  │
│   │ REP 线程     │   │ PUB 线程      │   │ 控制线程 (100Hz 主循环) │  │
│   │ 命令处理     │   │ 50Hz 状态     │   │ 单写线程, 串行所有串口写 │  │
│   └──────┬──────┘   └──────┬───────┘   └─────┬────────────────┘  │
│          │                 │  共享状态(mutex) │                   │
│          └─────────────────┴─────────────────┘                   │
│                        │ 每 tick:                               │
│                        │  1. 写 4 帧编码器读 (一次 write)          │
│                        │  2. ≤3ms 收应答 -> 积分里程计            │
│                        │  3. 模式控制 (VELOCITY/POSITION/IDLE)   │
│                        │  4. 轮速有变化才下发 motor4 帧           │
│                        │  5. 帧间隙发透传帧 (TX 队列)             │
└────────────────────────┼─────────────────────────────────────────┘
                         │ /dev/ttyUSB0 (1,000,000 8N1 raw, TIOCEXCL)
                    ┌────▼────┐
                    │  MC602  │── 4 路电机 + 4 路编码器 + 其他设备
                    └─────────┘
```

## 2. ZMQ API

### REP `tcp://127.0.0.1:6010`（JSON）

| 命令 | 请求 | 应答 |
|---|---|---|
| vel | `{"cmd":"vel","v":[x,y,z]}` | `{"ok":true}` 切 VELOCITY，喂看门狗 |
| goto | `{"cmd":"goto","target":[x,y,th],"max_v":[..],"tol":[..],"timeout":30}` | `{"ok":true}` 异步启动 POSITION；`duration` 可选，给出时 `max_v[i]=abs(target[i]-cur[i])/duration` |
| cancel_goto | `{"cmd":"cancel_goto"}` | `{"ok":true}` 取消闭环并停车 |
| stop | `{"cmd":"stop"}` | `{"ok":true}` 零速 + IDLE |
| reset | `{"cmd":"reset","x":..,"y":..,"z":..,"distance":..}` | `{"ok":true}` 里程计 reset，字段缺省=保持原值 |
| state | `{"cmd":"state"}` | 直接返回状态对象（无 ok 包装） |
| frame | `{"cmd":"frame","payload":"<hex>","timeout_ms":200}` | `{"ok":true,"payload":"<hex>"}` 或 `{"ok":false}`；同步透传，等 (payload[0],payload[1],payload[2]) 匹配的应答 |
| frame_async | `{"cmd":"frame_async","payload":"hex","seq":N}` | `{"ok":true}`；应答/超时走 PUB |

`state` 字段：`x`,`y`,`th`,`dist`,`mode`（`idle`/`velocity`/`position`）,`goto_active`,`goto_ok`,`tick_err_ms`。

### PUB `tcp://127.0.0.1:6011`（JSON，50Hz 状态 + 事件）

- 50Hz：`{"evt":"state", ...同 state}`
- 事件：`{"evt":"reply","seq":N,"payload":"hex"}` / `{"evt":"timeout","seq":N}`

## 3. 串口帧

线路帧：`77 68 <total_len> <payload...> 0A`，`total_len = len(payload)+4`。应答帧同格式。

| 帧 | payload（小端） | 说明 |
|---|---|---|
| motor4 轮速 | `01 02 <s1> <s2> <s3> <s4>`，每个 s 为 **int8** | dev=0x01, mode=2(set)。**无 port 字节**（Motor4_2 未设 port_id，get_bytes 的 port else 分支不补 0）。4 个轮速全部上线 |
| 编码器读 | `04 01 <port> 00 00 00 00`（port=1..4） | 4 帧拼一次写；每帧含 4 字节 int32 占位 |
| 编码器应答 | `04 01 <port> <int32 LE>` | 各自独立应答，按 (dev,mode,port) 匹配 |
| 透传帧 | `<dev> <mode> <port> <data...>` | 由 ZMQ frame/frame_async 原样打包 |

**应答匹配键** = `(payload[0], payload[1], payload[2])` = (dev, mode, port)。注意 motor4 帧无 port 字节，其 payload[2] 是 s1（首个轮速）——匹配键照旧用 payload[2]，与 Python 引擎行为一致。

### 轮速换算（复刻 WheelWrap/Motors，reverse=false）

```
virtual_i = clip(wheel_linear_i * (1/0.03) * 3.204498, -100, 100)  # 截断取 int8
wire_i    = -virtual_i                                             # 全部取负 (reverse=false 语义)
payload   = 01 02 wire_1 wire_2 wire_3 wire_4                      # 全部上线
```

- `rad2virtual = 48*(28/11)^4 / (2π) / 100 = 3.204498`（现场标定值）
- 例：`vx=0.1` → 逆解 `[0.1,-0.1,-0.1,0.1]` → virtual `[10,-10,-10,10]` → 取负 `[-10,10,10,-10]` → 线路 `77 68 0a 01 02 f6 0a 0a f6 0a`
- ⚠️ 曾有"丢第一个轮速"的旧说法：那是早期把 `port_id=0` 显式传入 `get_bytes` 时出现的形态（`01 02 00 e0 df 22`）。真实链路 `Motors_2.set_speed → motor4.set_speed → get_bytes(..., port_id=None)` **不丢**，4 个轮速全上线。已用真实 `DevCmdInterface` 复算验证（`set_speed([31,-32,-33,34]) → 01 02 1f e0 df 22`）。

### 编码器→位移（复刻 WheelWrap.get_linear，reverse=false）

```
linear_disp_i = -enc_raw_delta_i * (1/320.44975) * 0.03
```

编码器原始值取负——与轮速侧取负成对，保证正转=正里程（现场标定行为，须保留）。

## 4. 运动学 / 里程计 / PID

- **MecanumChassis**（默认参数）：`roller_angle=π/4*1.052`，`tan=tan(roller_angle)`，`half_track=0.15`，`half_wheel_base=0.14`，`wc=0.15*tan+0.14`
  - 逆解：`wheel0= vx+vy*tan+wz*wc; wheel1=-vx+vy*tan+wz*wc; wheel2=-vx-vy*tan+wz*wc; wheel3= vx-vy*tan+wz*wc`
  - 正解：`dx=(d0-d1-d2+d3)/4; dy=(d0+d1-d2-d3)/(4*tan); dth=(d0+d1+d2+d3)/(4*wc)`
- **Odometry.update**（车辆系→世界系）：`dx'=dx*cosθ-dy*sinθ; dy'=dx*sinθ+dy*cosθ; distance+=hypot(dx,dy)`
- **PID**（simple-pid 语义，`differential_on_measurement=true`）：`sample_time=0.01`（dt<sample_time 且有上次输出→直接返回上次输出）；`derivative=-Kd*(input-last_input)/dt`；积分 clamp 到 output_limits、误差符号翻转时积分清零；`_last_error` 初始 0。三个实例持久存在（跨 goto 保留状态）：`pid_x(Kp=6,Ki=0.3,Kd=0.1,±0.6)`、`pid_y(8,0.3,0.1,±0.6)`、`pid_yaw(10,0.2,0.1,±1.5)`
- **move_to_position 闭环**：进入 POSITION 时按 duration 或 max_v 设 output_limits、setpoint=target；每 tick 先查超时(默认30s)/迭代>1000 → 结束(ok=false)；航向归一化 `target[2]+2π*round((cur[2]-target[2])/2π)`；三项误差全 < tol(默认 [0.004,0.004,0.02]) 则 consecutive++，>20 → 结束(ok=true)；然后 `vx=pid_x, vy=pid_y, pid_yaw.setpoint=target_theta, wz=pid_yaw`；世界→车旋转 `vcx=vx*cosθ+vy*sinθ; vcy=-vx*sinθ+vy*cosθ`；逆解下发。结束必发零速。

## 5. 控制主循环（100Hz）

- `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` 绝对时间节拍，无累积漂移；`--tick-hz` 可配
- 单写线程串行所有串口写
- 每 tick：写 4 帧编码器读 → ≤3ms（`--recv-budget`）收应答积分 → 模式控制 → 轮速有变化才下发
- **看门狗**：VELOCITY 下最近 vel 命令超 0.5s → 自动零速
- 透传帧在控制帧间隙发送（TX 队列，优先级低于控制帧）
- 编码器应答超 1s 未到：记日志，里程计冻结不崩
- SIGINT/SIGTERM：强制零速 → 关串口 → 退出

## 6. 构建

```bash
cd cpp/rtd
mkdir build && cd build
cmake ..          # 需要 g++11+ / cmake / libzmq / cppzmq(zmq.hpp) / nlohmann-json
make
```

产物 `build/rtd`。构建时 `kinematics_selftest()` 会校验轮速换算/帧字节与 Python 复算一致（断言失败即退出）。

### 联调（不开真实串口）

```bash
./rtd --simulate --tick-hz 100
```

`--simulate` 用虚拟设备（不碰 `/dev/ttyUSB0`）：编码器按"最近下发的轮速"回放，里程计在模拟下跟随指令，可验证 vel/goto/reset/frame 闭环。冒烟测试脚本随仓库提供：

```bash
./rtd --simulate --tick-hz 100 &   # 后台起守护进程
python3 cpp/rtd/smoke_test.py      # 跑 11 项 ZMQ 往返检查
```

## 7. 上车部署（systemd）

1. 停止 Python 控制进程（rtd 用 `TIOCEXCL` 独占串口，占用会失败）
2. 编译 `build/rtd` 放到 `/usr/local/bin/rtd`
3. 安装单元文件并启用：

```bash
sudo cp scripts/rtd.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rtd
journalctl -u rtd -f      # 看日志
```

单元示例见 `scripts/rtd.service`。

## 8. 安全与回退

- **绝不在 Python 进程仍占用串口时启动 rtd**（TIOCEXCL 会失败；若强行用 `--simulate` 之外的参数启动且串口被占用，rtd 会报错退出，不碰硬件）
- 看门狗三重保险：VELOCITY 0.5s 自动零速、编码器超 1s 冻结不崩、SIGTERM 强制零速
- 回退：`sudo systemctl stop rtd` 后即可重新启动原 Python 流程；rtd 没有改动任何现有文件，可随时整体删除 `cpp/rtd/`
- 联调端口 6010/6011 为本地回环，无外部暴露

## 9. 遗留风险 / 未覆盖项

- **encoder4（dev 0x03）** 与单路 `encoder`（dev 0x04）的差异：本实现按任务规格走 dev 0x04 四路独立读；若现场用 dev 0x03 合并帧，需改 `mc602::encoder_read_all`
- 透传帧若恰好是 `04 01 1..4`（编码器键），其应答会被编码器收集器优先消费（与 Python 引擎一致，属协议固有歧义）
- `round` 用 `std::round`（half-away-from-zero），Python `round` 是 half-even；只在角度差恰好 .5 边界有差异
- 节拍为软实时（Linux 非 PREEMPT_RT），极端负载下 `tick_err_ms` 会增大；里程计/PID 不依赖精确 dt 累积（与 Python 相同）
- motor4 的应答帧（若有）未等待，只透传路由；Python 侧 `set()` 会等应答，行为差异已按"有变化则下发、不等应答"的规格取舍
