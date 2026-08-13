# 百度智能车 2026 项目

## 项目简介

本项目基于百度 PaddlePaddle 深度学习平台和 WhalesBot 智能车硬件平台开发，实现了完整的智慧农业赛道竞赛任务。智能车能够自主完成播种、灌溉、射击除害、作物采收、分类储存及订单配送等一系列农业自动化任务。

## 硬件平台

| 组件 | 说明 |
|------|------|
| 上位机 | Jetson Nano |
| 下位机 | MC602 控制器 |
| 底盘 | WhalesBot 麦克纳姆轮底盘 |
| 前置摄像头 | 车道检测（LaneInfer 模型） |
| 侧面摄像头 | 目标检测（YoloeInfer 模型） |
| 机械臂 | 夹取、放置、气泵吸取 |
| 其他 | 蜂鸣器、储物架、射击机构 |

## 项目结构

```
baidu_smartcar_2026/
├── car_start_2026.py        # 主启动脚本，执行完整任务流程
├── car_task_function.py     # 任务函数（8个竞赛任务）
├── car_wrap_2026.py         # MyCar 核心控制类
├── config_car.yml           # 硬件参数、PID参数、推理服务配置
├── collect_data.py          # 数据采集脚本
├── smartcar/
│   ├── paddlebaidu/
│   │   ├── ernie_bot/       # 文心一言 API 封装
│   │   ├── infer_cs/        # 推理服务接口
│   │   ├── models/          # 视觉模型（task2026、front_model2 等）
│   │   └── paddle_jetson/   # Jetson 平台部署工具
│   └── whalesbot/
│       ├── vehicle/         # 车辆底层驱动
│       └── tools/           # PID、摄像头、日志等工具
└── README.md
```

## 技术栈

- **编程语言**：Python 3
- **深度学习推理**：百度 PaddlePaddle Inference
- **目标检测模型**：YoloeInfer（任务目标）、LaneInfer（车道线）
- **OCR 识别**：百度 PP-OCRv3
- **自然语言处理**：百度文心一言 API
- **控制算法**：麦克纳姆轮运动控制、PID 控制
- **硬件接口**：WhalesBot SDK

## 任务流程

`car_start_2026.py` 中的 `main()` 函数按顺序执行以下8个任务：

```
init()                      → 系统初始化（机械臂复位、里程计清零）
    ↓
auto_seeding()             → 自动播种（3个大/中/小圆柱体）
    ↓
target_shooting_detection() → 虫害侦察（识别4个靶标，判断有害/有益动物）
    ↓
water_tower_task()         → 水塔灌溉（2个水塔，各需不同水量）
    ↓
target_shooting()          → 射击除害（在射击区击倒有害动物靶标）
    ↓
crop_harvesting()          → 作物采收（2种颜色×4个果实，吸取到储物架）
    ↓
sort_and_store()           → 分类储存（按颜色分拣到存储仓）
    ↓
get_order()                → 获取订单（OCR识别+大模型解析，按序取货）
    ↓
order_delivery()           → 订单配送（将货物送到对应住户格口）
```

## 目标检测标签

程序中使用以下目标检测标签进行视觉识别：

| 类别 | 说明 |
|------|------|
| cylinder_1/2/3 | 播种用圆柱体（大/中/小） |
| water_l1/l2/l3 | 水塔水量指示牌（1/2/3滴水） |
| ball_yellow / ball_blue | 果实（黄色/蓝色） |
| lable_yellow / lable_blue | 颜色标签 |
| animal | 动物靶标 |
| order | 订单标签 |
| name | 住户姓名标签 |
| storage | 储物架 |

## 安装配置

### 1. 环境要求

- Python 3.8+
- PaddlePaddle Inference（需配合 Jetson Nano 的 CUDA 版本）
- WhalesBot SDK

### 2. 配置文件

编辑 `config_car.yml` 中的关键参数：

```yaml
# 摄像头通道
camera:
  front: 1    # 前视摄像头
  side: 2     # 侧视摄像头

# 速度限制
speed:
  x:
    limit: 0.7    # 横向 m/s
  y:
    limit: 0.7    # 纵向 m/s
  angle:
    limit: 3      # 角速度 rad/s
```

### 3. 文心一言 API

访问令牌已配置在 `config_car.yml` 的 `ernie_access_token` 字段。

## 使用方法

### 完整任务流程

```bash
python car_start_2026.py
```

### 单独测试某个任务

在 `car_start_2026.py` 中注释掉不需要的任务：

```python
def main():
    init()
    auto_seeding()                          # 保留要测试的任务
    # target_shooting_detection()            # 注释掉其他任务
    # water_tower_task()
    # ...
```

## 核心类与方法

### MyCar 类（car_wrap_2026.py）

核心控制类，继承麦克纳姆轮驱动，提供所有硬件操作接口：

| 方法 | 说明 |
|------|------|
| `beep()` | 鸣笛提示 |
| `reset_position()` | 复位里程计 |
| `get_odometry()` | 获取当前坐标 (x, y, z) |
| `move_for([dx, dy, dz])` | 相对移动 |
| `lane_dis_offset(speed, dis_hold)` | 巡线行驶指定距离 |
| `move_to_position(coords)` | 移动到绝对坐标 |
| `move_to_detection_target()` | 视觉对准目标 |
| `get_detection_results()` | 获取检测结果列表 |
| `get_ocr(label, time_out)` | OCR 文字识别 |
| `set_storage(state)` | 控制储物架抬升/下降 |

### 机械臂操作

| 方法 | 说明 |
|------|------|
| `arm.reset_position()` | 机械臂复位 |
| `arm.set_arm_pose()` | 设置机械臂姿态（左右臂、俯仰角） |
| `arm.grasp(state)` | 气泵吸取/释放（True=吸气，False=放气） |
| `arm.move_x_position()` | 机械臂 X 轴移动 |
| `arm.move_y_position()` | 机械臂 Y 轴移动 |

## 推理服务

项目使用 ZMQ 通信的后端推理服务，各模型独立运行在不同端口：

| 模型 | 类型 | 端口 | 说明 |
|------|------|------|------|
| lane | LaneInfer | 5001 | 车道线检测 |
| task | YoloeInfer | 5002 | 任务目标检测 |
| front | YoloeInfer | 5003 | 前方目标检测 |
| ocr | OCRReco | 5004 | 文字识别 |

推理服务随 MyCar 初始化时自动启动，无需手动启动。

## 重要说明

本程序仅对任务可行性做了初步验证，并未针对不同赛场的差异性进行全面测试。项目中的任务解法不一定是全局最优解，实际比赛中需要选手根据各自的场地条件进行调试和优化。里程计参数、视觉识别的阈值、机械臂位置偏移等均需结合实际情况校准，必要时可对函数逻辑进行大幅调整甚至重写，以增强程序在不同环境下的鲁棒性和运行效率。

### 数据采集控制

运行 `smartcar/whalesbot/tools/collect_control.py` 启动双摄像头数据采集系统：

```bash
python -m smartcar.whalesbot.tools.collect_control
```

系统自动启动局域网双摄流媒体服务，打开浏览器访问：

```
http://<智能车IP>:5000/
```

可同时查看 cam1（车道摄像头）和 cam2（目标摄像头）的实时画面，支持网页键盘事件反馈显示。

**遥控器按键功能表：**

| 按键 | 功能 |
|------|------|
| **左摇杆** | 控制智能车在 X/Y 方向移动 |
| **右摇杆左右** | 控制车身旋转 |
| **△** | 机械臂向上移动 |
| **▽** | 机械臂向下移动 |
| **◁** | 机械臂向左移动 |
| **▷** | 机械臂向右移动 |
| **^** | 机械臂手爪向上 |
| **V** | 机械臂手爪向下 |
| **<** | 机械臂整体向左 |
| **>** | 机械臂整体向右 |
| **□** | 切换气泵吸取/释放 |
| **○** | 切换舵机角度 |
| **按键3** | cam1（车道摄像头）开始采集数据 |
| **按键4** | cam2（目标摄像头）开始采集数据 |
| **L1+V** | 删除 cam1 最近 30 张图片 |
| **L1+O** | 清空 cam1 所有数据 |
| **L2+▽** | 删除 cam2 最近 30 张图片 |
| **L2+□** | 清空 cam2 所有数据 |
| **L1+L2** | 安全退出程序 |

**摄像头配置：**
- cam1：前置车道摄像头（320×240），保存到 `./dataset/image_set_lane/`
- cam2：侧向目标摄像头（640×480），保存到 `./dataset/image_set_object/`

## 故障排查

| 现象 | 可能原因 |
|------|---------|
| 摄像头读取失败 | 摄像头通道配置错误或连接不良 |
| 巡线偏离车道 | 光照变化或场地纹路干扰；尝试降低巡线速度 |
| 目标检测失败 | 摄像头角度偏差；目标不在视野范围内 |
| 机械臂抓取失败 | 气泵管路漏气；位置偏差导致未对准目标 |
| 大模型解析失败 | 网络波动；API 超时 |

## 贡献

欢迎提交 Issue 或 Pull Request 对项目进行改进。

## 许可证

MIT License
