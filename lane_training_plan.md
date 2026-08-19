# 车道巡航 CNN 训练方案

> 目标:用 `merged_dataset` 重训/改进 `lane_model`(示例 A 的 CnnModel),产出新的
> TRT 引擎部署到 Jetson。本文档整合数据集质量评估 + 训练策略 + 部署链路。

---

## 1. 数据集质量评估

### 1.1 总览

| 项 | 值 |
|---|---|
| 路径 | `D:\桌面\智能车比赛\merged_dataset\merged_dataset\` |
| 总数 | 37,319 张(train 33,587 + val 3,732) |
| 划分 | 9:1,seed 42 |
| 标注格式 | list of `{img_path, state}`,符合示例 A |
| 图像 | 640×480 RGB |
| 完整性 | 0 缺失 |

### 1.2 标签结构(state 三元组)

`state = [speed, error, angle]`,来自 `collect_control.py` 的手柄状态:

| index | 含义 | 实际值 | 结论 |
|---|---|---|---|
| `state[0]` | 速度档位 | 恒 0.3 | 训练时丢弃(`label[1:]`) |
| `state[1]` | 横向误差(左摇杆X) | **恒 0** | 采集时没碰左摇杆,error 分支死 |
| `state[2]` | 转角(右摇杆Y) | [-0.64, 1.55],mean 0.239 | **唯一有信号的输出** |

- 示例 A 模型输出 2 维 `(error, angle)`,error 恒 0 → 学成 0,无害(推理侧有
  `correction_model` 等其他方案替代 error,见 memory `rules-vs-notes-vs-code-conflicts`)。
- **转角范围不对称**(正最多 1.55、负最多 -0.64)是**左转多右转少**造成的,不是刻度问题。

### 1.3 类别分布(逆时针赛道,符合预期)

| 方向 | angle 符号 | 占比 |
|---|---|---|
| 左转 | > 0 | **72.3%** |
| 右转 | < 0 | 27.6% |
| 直行 | ≈0(\|angle\|<0.05) | 35.8% |

- 逆时针赛道本就左转主导,**不是采集错误**。
- 右转偏少 → 训练时**开 hflip**(左转镜像成右转,右转样本翻倍)兜底。

### 1.4 一个存疑点:`image_sets_s1..s5`(约 1400 张)

- 标签几乎全 0(最大 0.056,stdev ≤0.016),即全是"直行"。
- 黄线检测显示车道线严重偏向一侧,**疑似"车在转弯但手柄信号没读到、标签误记为 0"**。
- **低风险**:仅占 3.7%,且主数据直行样本已 35.8%,模型扛得住。
- 结论:**可删可不删**。想干净就用 `train_cnn_lane.py --drop-dead` 过滤。

### 1.5 一个已知局限:val 有相邻帧泄露

val 是 session 内随机切分,同一 session 时间相邻的帧可能进 train/val 两边,
**val 指标偏乐观**,只用于判断收敛,不当最终真相。

---

## 2. 训练策略

### 2.1 在哪训

**AI Studio 云端(V100/A100 + paddle 2.x),本地不训。**

本地(Windows)不可训的原因:
- Python 3.12.7 + CPU paddle,`import paddle` 直接崩(protobuf 版本冲突);
- RTX 5060 是 Blackwell(sm_120),paddle 官方 wheel 大概率不支持。

### 2.2 训练脚本

`scripts/train_cnn_lane.py`(已就绪)。

```bash
python train_cnn_lane.py \
    --data /home/aistudio/data/<数据集ID>/merged_dataset \
    --epochs 300 --batch 256 \
    --out /home/aistudio/model/cnn23 \
    --drop-dead --export
```

### 2.3 超参

| 项 | 值 | 说明 |
|---|---|---|
| 模型 | CnnModel(6conv+3fc,输出2) | 照抄示例 A |
| 输入 | 128×128 RGB,`x/127.5-1` | **严格对齐 TrtLaneInfer** |
| loss | L1Loss(MAE) | 示例 A 同款 |
| 优化器 | Adam | |
| lr | PiecewiseDecay `[100,200]`→`[1e-3,1e-4,1e-5]` | 数据 6 倍于示例 A |
| 增强 | 5选1(hue/sat/contrast/bright/hflip) | hflip 兜底右转,别关 |
| epoch | 300 | 示例 A 200 不够 |

---

## 3. 部署链路

```
AI Studio 训练
  └─ 动转静导出 cnn_lane.pdmodel + .pdiparams(输入名 inputs)
       ↓ 下载覆盖到 rak/smartcar/paddlebaidu/models/lane_model/
  paddle2onnx → lane.onnx(opset 13)
       ↓
  bash scripts/build_trt_engines.sh --rebuild  (在 Jetson 上跑 trtexec)
       ↓
  trt_engines/lane_fp16.engine 生成, 推理自动用新引擎
```

`build_trt_engines.sh` 已配好 lane 这条(`build_one lane lane_model cnn_lane.pdmodel ...
"inputs:1x3x128x128"`),**覆盖权重后 `--rebuild` 即可,不用改脚本**。

---

## 4. 三个必须记住的坑

1. **预处理必须 `x/127.5 - 1.0`**(范围 [-1,1] 的 RGB)。训练脚本已写死,别改成
   paddle 的 `Normalize+ToTensor` 套写法(虽最终等价,但显式写更稳)。
2. **输入名必须 `inputs`**,否则 `TrtEngine.infer({"inputs": x})` 找不到张量。
3. **训练在 AI Studio,不在本地**。

---

## 附:示例 A 脚本的已知问题(不要直接照抄)

| 问题 | 说明 |
|---|---|
| `load_alldata()` 内存爆炸 | 3.7 万张全读内存 ≈ 7GB+,你数据量大不能照抄 |
| 不存 best model | 最后存的是 epoch 199,不一定最好 |
| lr boundaries 超界 | `[100,400]` 但 epochs 200,第三段永不用 |
| 日志爆炸 | 每 batch print loss |

→ 这些坑 `train_cnn_lane.py` 已全部避开。
