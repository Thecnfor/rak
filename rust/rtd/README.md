# rtd —— 实时控制守护进程 (Rust 版)

接管 `/dev/ttyUSB0`(MC602 控制器), 以固定 100Hz 做: **读编码器 → 积分里程计 →
速度/位置闭环 PID → 下发轮速**。Python 侧通过 ZMQ 下发命令、并透传其他设备帧
(机械臂/舵机/按键/蜂鸣器走同一串口)。

本实现与 `/home/xrak/rak/cpp/rtd/`(C++ 版)是**同协议姊妹实现**, 可互换部署;
语义逐条复刻 `smartcar/whalesbot/` 下的 Python 参考实现。

> ⚠️ **安全**: 默认连接真实串口。**联调用 `--simulate`(不开串口, 虚拟设备),
> 上车前务必确认 Python 进程已停止**(见文末"部署切换")。

---

## 1. 架构

```
                ┌───────────────────────────────────────────────┐
  Python 进程    │  rtd (Rust 守护进程)                            │
  SMARTCAR_RTD=1│                                                 │
 ┌───────────┐  │  ┌───────────────┐   ┌──────────────────────┐  │
 │ Mecanum    │  │  │ REP 线程 (std) │──▶│ 共享核心 Mutex<Core>  │  │
 │ Driver /   │──▶│  zmq 6010      │◀──│ 模式/里程计/PID/goto   │  │
 │ serial_wrap│  │  └───────────────┘   │                       │  │
 └───────────┘  │  ┌───────────────┐   │ ┌──────────────────┐  │  ┌───────────────┐
      │  ZMQ    │  │ 控制线程 (std) │──▶│ │ 100Hz 绝对节拍主循环 │  │  │  MC602 /dev/   │
      │ 6010/   │  │ 100Hz         │   │ │ 编码器读/应答路由    │──▶│  ttyUSB0       │
      │ 6011    │  └───────────────┘   │ │ 里程计积分/闭环      │  │  1000000 8N1   │
                │  ┌───────────────┐   │ └──────────────────┘  │  └───────────────┘
                │  │ PUB 任务(tokio)│◀──│ 应答事件/50Hz 状态     │
                │  │ zmq 6011      │   └──────────────────────┘
                │  └───────────────┘
                └───────────────────────────────────────────────┘
```

- **控制线程**(std, 名 `rtd-control`): 唯一做串口读写 + 积分/闭环的线程, `clock_nanosleep
  TIMER_ABSTIME` 绝对节拍, 不漂移。透传帧在控制帧间隙发送(优先级低)。
- **REP 线程**(std, 名 `rtd-rep`): 阻塞 zmq REQ/REP, 改共享状态; `frame` 命令经
  Condvar 等控制线程路由应答(同步一问一答, 与 Python `get_anwser` 语义一致)。
- **PUB 任务**(tokio async): 50Hz 状态心跳 + 事件(应答/超时/订阅帧)即时推送;
  SIGINT/SIGTERM 也由 tokio 信号驱动捕获, 平滑停机。
- 共享状态全部由 `Mutex<Core>` 保护; 锁序约定"只允许 core→io 嵌套", 应答路由与
  等待方先释放 core 锁再拿 waiter 锁, 避免死锁。

### 模块划分

| 文件 | 职责 | 对应 Python / C++ 参考 |
|------|------|------------------------|
| `src/main.rs` | CLI 解析 / 生命周期 / 信号 | `cpp/rtd/src/main.cpp` |
| `src/proto.rs` | MC 帧编解码 / 切帧器 / 命令帧构造 | `serial_protocol.py` / `protocol.{h,cpp}` |
| `src/kin.rs` | 正逆解 / 轮速换算 / 编码器→位移(常量集中) | `mecanum.py` / `kinematics.h` |
| `src/pid.rs` | simple-pid 语义(微分先行, 积分防饱和) | `tools_class.py::PID` / `pid.h` |
| `src/odom.rs` | 里程计积分 / reset | `mecanum.py::Odometry` |
| `src/io.rs` | **crate 内唯一 unsafe 模块**: termios 串口 + 绝对节拍睡眠 + 虚拟设备 | `serial.cpp` |
| `src/core.rs` | 100Hz 主循环 / 状态机 / goto 闭环 / 应答路由 / 命令处理 | `rtd.cpp::RtdCore` |
| `src/zmq.rs` | REP 线程 / PUB 任务 | `zmq_api.cpp` |
| `src/util.rs` | 单调时钟 / hex / 小端 / 日志 | `util.{h,cpp}` |

---

## 2. 串口与帧协议

- `/dev/ttyUSB0`, **1,000,000 波特 8N1 raw**(termios `B1000000`, `CRTSCTS` off,
  `ICANON` off), `O_RDWR | O_NOCTTY` + `ioctl(TIOCEXCL)` 独占。
- 帧: `77 68 <total_len> <payload...> 0A`, `total_len = payload+4`(单字节); 应答同格式。
- payload = `<dev_id> <mode> <port> <data...>`(小端)。
- 命令帧:
  - **motor4 轮速**: `01 02 <w1> <w2> <w3> <w4>`(4 个 int8, **无 port 字节** —
    `get_bytes` 的 port else 分支不补 0)。示例 `[31,-32,-33,34]` → `77 68 0a 01 02 1f e0 df 22 0a`。
  - **编码器读**: 4 帧拼一次写, 每帧 `04 01 <port> 00 00 00 00`(port=1..4, 含 port 字节);
    应答 `04 01 <port> <int32 LE>`, 按 `(dev,mode,port)` 匹配。
- 收流切帧(复刻 `parse_mc_stream`): 帧头不对跳 1 字节; 长度不足等待; 帧尾不对丢弃该帧头继续扫。

---

## 3. 运动学 / 里程计 / PID

所有数值常量集中在 `src/kin.rs` / `src/core.rs`, 注明来源公式(复刻现场标定):

- `tan = tan(π/4·1.052)`; `wc = 0.15·tan + 0.14`
- 逆解: `wheel0= vx+vy·tan+wz·wc; wheel1=-vx+vy·tan+wz·wc; wheel2=-vx-vy·tan+wz·wc; wheel3= vx-vy·tan+wz·wc`
- 正解: `dx=(d0-d1-d2+d3)/4; dy=(d0+d1-d2-d3)/(4·tan); dth=(d0+d1+d2+d3)/(4·wc)`
- 轮速换算(reverse=false): `int8 = clip(wheel_linear·(1/0.03)·3.204498, -100, 100)`, 上线前**全部取负**
- 编码器→位移: `linear_disp_i = -enc_raw_i·(1/320.44975)·0.03`(用增量 cur-prev)
  - 取负与轮速取负成对 —— **复刻现场标定行为**, 保证"正转=正里程"。
- 里程计积分: `dx'=dx·cosθ-dy·sinθ; dy'=dx·sinθ+dy·cosθ; distance+=hypot(dx,dy)`
- PID(复刻 simple-pid, `sample_time=0.01`, 微分先行): P=`Kp·(setpoint-input)`;
  I+=`Ki·error·dt` 并 clamp 到 output_limits, **误差符号翻转时 I 清零**;
  D=`-Kd·(input-last_input)/dt`(首次 d=0); 输出 clamp。三个实例持久存在(跨 goto 保留):
  `pid_x(6,0.3,0.1,±0.6)`, `pid_y(8,0.3,0.1,±0.6)`, `pid_yaw(10,0.2,0.1,±1.5)`。

---

## 4. ZMQ API(与 C++ 版完全同协议)

**REP** `tcp://127.0.0.1:6010`(JSON):

| 命令 | 说明 |
|------|------|
| `{"cmd":"vel","v":[x,y,z]}` | 设速度(世界系), 模式→velocity; 需持续喂以喂饱 0.5s 看门狗 |
| `{"cmd":"goto","target":[x,y,th],"max_v":..,"tol":..,"timeout":30}` | 异步启动位置闭环, 完成看状态; `duration` 可选 |
| `{"cmd":"cancel_goto"}` / `{"cmd":"stop"}` | 取消 goto / 停车, 模式→idle |
| `{"cmd":"reset","x":..,"y":..,"z":..,"distance":..}` | 重置里程计,**字段缺省=保持原值** |
| `{"cmd":"state"}` | → `{"x","y","th","dist","mode","goto_active","goto_ok","tick_err_ms"}` |
| `{"cmd":"frame","payload":"<hex>","timeout_ms":200}` | 同步透传, 回 `{"ok":true,"payload":"<hex>"}` / `{"ok":false}` |
| `{"cmd":"frame_async","payload":"<hex>","seq":N}` | 立即回 `{"ok":true}`, 应答/超时走 PUB |
| `{"cmd":"sub","dev":d,"mode":m,"port":p}` | 订阅该 `(dev,mode,port)` 帧, 走 PUB `evt:frame` |

**PUB** `tcp://127.0.0.1:6011`:
- 50Hz `{"evt":"state", ...同 state}`
- `{"evt":"reply","seq":N,"payload":"<hex>"}` / `{"evt":"timeout","seq":N}`
- `{"evt":"frame","dev":..,"mode":..,"port":..,"payload":"<hex>"}`

---

## 5. 构建

```bash
source ~/.cargo/env
cd /home/xrak/rak/rust/rtd
cargo build --release          # 或: cargo build --release -j 1 (内存紧张时)
```

依赖: `libzmq3-dev` + `pkg-config`(zmq-sys 走系统 libzmq)、`serde_json`、`libc`、`tokio`
(仅 PUB/信号)。`.cargo/config.toml` 已设 `jobs=1` + `codegen-units=1` 降编译内存
(本机 ~3.5G 且大部分被占)。产物: `target/release/rtd`(~1MB)。

---

## 6. 命令行

```
rtd [--port PATH] [--tick-hz N] [--cmd-port N] [--pub-port N] [--simulate] [--recv-budget MS]
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `--port` | `/dev/ttyUSB0` | 串口设备 |
| `--tick-hz` | `100` | 控制频率(10..1000) |
| `--cmd-port` / `--pub-port` | `6010` / `6011` | ZMQ REP / PUB 端口 |
| `--simulate` | 关 | 不开串口, 虚拟设备联调(编码器按下发轮速回放, 闭环可跑通) |
| `--recv-budget` | `3` | 每 tick 编码器应答接收预算 ms |

---

## 7. 冒烟测试(安全, 不碰硬件)

```bash
./target/release/rtd --simulate &      # 6010/6011 为安全端口
# Python zmq 客户端: state / vel / reset / frame / frame_async / sub / goto 逐项验证
# 见仓库 scripts/ 下参考脚本(或自行按第 4 节协议编写)
kill -TERM <pid>                       # 平滑停机: 零速 -> 关串口 -> 各线程退出
```

---

## 8. 部署切换(与 C++ 版互换)

Python 侧启用 rtd 模式: 环境变量 `SMARTCAR_RTD=1`(`SMARTCAR_RTD_CMD_URL` /
`SMARTCAR_RTD_PUB_URL` 可覆盖端口)。此时 `serial_wrap` 与 `MecanumDriver` 全部走
ZMQ 透传/委托, 与守护进程版本无关。

```bash
# 1) 停 Python 任务(避免与 rtd 争抢 /dev/ttyUSB0)
# 2) 启动 Rust 版(不设 --simulate!)
./target/release/rtd --port /dev/ttyUSB0 --tick-hz 100 --cmd-port 6010 --pub-port 6011
# 3) 跑任务: SMARTCAR_RTD=1 python3 run.py all
```

建议用 systemd 管理(崩溃自动重启): 见 `scripts/rtd-rust.service`。

**回退 C++ 版**: 停 Rust 版, 启动 `cpp/rtd/build/rtd`(同参数), Python 侧零改动。
两个版本协议一致, 可随时切换。

---

## 9. 安全与回退

- **看门狗**: VELOCITY 下最近 vel 命令超 0.5s → 自动零速; 编码器应答超 1s 未到 →
  记日志"里程计冻结"不崩(5s 节流); SIGINT/SIGTERM → 零速 → 关串口 → 平滑退出。
- **回退**: 任何异常退出(非零速)都应在任务开始前检查。systemd `Restart=always`
  会拉起守护进程; Python 侧未设置 `SMARTCAR_RTD` 时走原串口路径(零行为变化)。
- 上车前请确认: 守护进程独占串口、Python 已停止、`SMARTCAR_RTD=1`。
