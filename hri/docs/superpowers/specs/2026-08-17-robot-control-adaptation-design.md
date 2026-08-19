# HRI GUI ↔ baidu_smartcar_2026 机器人控制适配 — 设计

日期：2026-08-17
状态：已批准（架构），实现中

## 目标

把 rak-hri 的 Qt/QML 比赛控制台（任务链、启动/停止、速度控制）真正接到
`/home/xrak/Desktop/baidu_smartcar_2026`（Jetson Orin 机器人核心控制），
用**网络**驱动 8 个真实比赛任务，替换当前占位状态翻转逻辑。

## 已确认决策

1. **通信**：网络。HTTP 命令 + WebSocket 状态推送（混合）。
2. **控制粒度**：两种都要 —— 全流程编排（一键 run_all）+ 单任务逐项触发。
3. **后端框架**：FastAPI + uvicorn。
4. **开发 mock**：要。无硬件时在 x86 预览机上跑模拟后端，同接口验证。

## 架构

```
rak-hri Qt/QML GUI                    baidu_smartcar_2026 控制后端
  AppController (C++)  ──HTTP──►        FastAPI server (asyncio)
  RobotClient (C++)    ◄──WS───         /api/* + /ws
  QML 组件（不变）                          │
                                         CarBackend 抽象层
                                           ├─ RealBackend（真 Orchestrator, 仅 Orin）
                                           └─ MockBackend（模拟 8 任务, 开发用）
```

- 后端在 Orin 上跑真车；开发期 `--mock` 在 x86 跑模拟。
- GUI 连接 host:port 可配置（localhost 或 Orin 局域网 IP）。

## 后端（baidu_smartcar_2026/control/）

新增 `control/` 包：

| 文件 | 职责 |
|---|---|
| `backend.py` | `CarBackend` 抽象基类 + 事件类型常量 |
| `real_backend.py` | 真后端：后台线程跑 Orchestrator；HTTP 命令→inject 按键(4/1/3)；状态→WS |
| `mock_backend.py` | 模拟后端：后台线程按 TASK_ORDER 走模拟任务（里程计递增、可跳过/急停） |
| `server.py` | FastAPI 应用：HTTP 命令端点 + `/ws` 推送 |
| `main.py` | CLI 入口：`python -m control.main [--mock] [--host] [--port]` |
| `requirements.txt` | fastapi, uvicorn |

### HTTP 端点

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/hello` | 快照（模式、任务、done、active） |
| GET | `/api/tasks` | 任务清单（name/status/speed/trigger） |
| GET | `/api/status` | 当前状态快照 |
| POST | `/api/start` | 一键启动 run_all；body 可选 `{from_index}` |
| POST | `/api/run/{task}` | 单任务触发 |
| POST | `/api/stop` | 急停（inject 按键 3） |
| POST | `/api/skip` | 跳过当前（inject 按键 1） |
| POST | `/api/reset` | 清空 done 集合（重来） |
| POST | `/api/tasks/{task}/speed` | 覆盖该任务触发速度 |

### WS 事件（`/ws`，JSON）

| 事件 | 负载 |
|---|---|
| `hello` | 同 `/api/hello` 快照 |
| `run:started` | — |
| `run:finished` | `{reason}` |
| `task:started` | `{task}` |
| `task:done` | `{task}` |
| `task:skipped` | `{task, reason}` |
| `task:error` | `{task, error}` |
| `odom` | `{x, y, dist}`（后台轮询推送） |

### 真后端（RealBackend）

- 复用现有 `Orchestrator`：HTTP 命令翻译成按键事件注入其队列（4=start/重来、1=skip、3=emergency）。
- 编排循环在后台线程跑，事件经线程安全回调推到 server 广播给 WS。
- `from_index`：先清 done，再把 `TASK_ORDER[0:index]` 标为 done，然后 run_all。
- 单任务：在编排线程内 `cruise_to_trigger(task) → before 钩子 → run_task_module(task) → after 钩子 → mark_done`，事件照发。
- 互斥：编排/单任务任一进行时拒绝新的 start/run（`active` 标志 + 锁）；stop/skip 始终允许。

### 模拟后端（MockBackend）

- 不依赖硬件/PaddlePaddle，可在 x86 跑。
- 模拟 car：按键队列（`get_btn` 弹注入键）、里程计递增、`stop/close/beep`。
- 后台线程按 TASK_ORDER（或 from_index 起）逐任务：发 `task:started` → 短延时模拟执行 → 发 `task:done`；期间响应 stop/skip。
- 触发描述照抄真实 trigger 表，便于 GUI 展示。

## GUI（rak-hri）

| 文件 | 改动 |
|---|---|
| `src/app/RobotClient.{h,cpp}` | **新增** C++ HTTP(QNetworkAccessManager) + WS(QWebSocket) 客户端；信号：connected/disconnected/eventReceived；方法：start/runTask/stop/skip/reset/setTaskSpeed/fetchHello |
| `src/app/AppController.{h,cpp}` | 改造成持 RobotClient：任务链换真实 8 任务（中文显示名+触发描述）；WS 事件→更新任务状态；startFrom/runSingle/stop/setTaskSpeed 转发给后端 |
| `src/main.cpp` | 配置后端 host/port（环境变量或默认 localhost） |
| `CMakeLists.txt` | 加 `Qt6::Network` + `Qt6::WebSockets` 组件 |
| QML | 基本不变（TaskChain/TaskCockpit 读 app 真实状态） |

### 任务中文映射

seeding→播种, target_detection→识别虫害, watering→灌溉, shooting→射击除害,
harvesting→作物收集, sorting→作物储存, ordering→订单获取, delivery→订单配送。

### GUI 行为映射

- `startFrom(index)` → `POST /api/start {from_index:index}`（index 前标 done，从 index 起跑）
- `runSingle(index)` → `POST /api/run/{task}`
- `stop()` → `POST /api/stop`
- `setTaskSpeed(index, v)` → `POST /api/tasks/{task}/speed {speed:v}`
- 后端状态经 WS 回灌 → GUI 任务 status（pending/running/done）实时更新

## 错误处理

- 后端连接失败：RobotClient 发 `connectionError`，GUI 顶栏/状态页显示「后端离线」，任务仍可点但操作被拒。
- 后端 busy（已有编排）：HTTP 返回 409，GUI 提示「已有任务运行中」。
- WS 断线重连：RobotClient 自动重连（指数退避），重连后重发 hello。
- 未知任务/参数：HTTP 400/404。

## 测试

- 后端：mock 模式下启动，用 curl/脚本验证各端点 + WS 事件序列。
- GUI：连 mock 后端，在 x86 预览机上完整走一遍交互（点任务、启动、跳过、急停、速度）。
- 真车：在 Orin 上连真后端做一次完整流程（需硬件，最后阶段）。
