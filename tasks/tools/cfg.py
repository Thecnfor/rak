# -*- coding: utf-8 -*-
"""巡线参数统一配置 —— 正式比赛代码与 scripts/lane/*.py 共用同一份.

集中管理: 转向 PID / 转向死区 / correction 叠加 / lane 滤波 / 前进速度.
这里改一处, 正式流程(lane_base / 推理线程)与调试脚本(lane.py / lane-stop.py)
同步生效, 避免参数漂移。

读取方式:
- 正式代码: 各模块 from ..tools.cfg import * 直接引用;
- 调试脚本: scripts/lane/lane_params.py 从本文件 re-export, 保持脚本 import 名不变。
"""
from __future__ import annotations

# ---------------- 转向 PID (角速度通道, da -> 角速度) ----------------
# 转向方向开关: 越修越偏/原地打转时改成 -1
STEER_SIGN = 1.0

# 转弯通道 PID(da -> 角速度)
PID_ANGLE_KP = 6.5 * STEER_SIGN  # 转向强度: 弯道转不过来就加大, 直道蛇形摆动就减小
PID_ANGLE_KI = 0.0  # 角通道积分: 一般不动
PID_ANGLE_KD = 0.0  # 阻尼: 抑制摆动; 摆动大->加大, 转向迟钝->减小
_a = 4.5
PID_ANGLE_LIMITS = (-_a, _a)  # 角速度限幅 (rad/s)

# ---------------- 转向死区 (da 进 PID 前) ----------------
# 直线时 da 也有 ~0.1 的噪声(实测: 直线~0.1, 45°转弯 0.4~0.6, 90°转弯 0.8~1.2),
# 死区把它削掉, 避免直线时方向微抖/蛇形。削零式: |da| 超过死区后按
# (|da|-PID_DEADZONE) 连续响应, 弯道转向量不突变。
# 直线仍抖 -> 加大; 弯道变迟钝 -> 减小。
PID_DEADZONE = 0.0


def lane_deadzone(da):
    """da 进角 PID 前的死区处理: 直线小噪声归零, 超过后连续响应(转弯不突变)."""
    if abs(da) <= PID_DEADZONE:
        return 0.0
    return (abs(da) - PID_DEADZONE) * (1.0 if da > 0 else -1.0)


# ---------------- 居中通道: correction steer 叠加到角速度 ----------------
# |steer| 低于此值不叠加(防抖)
CORR_THRESHOLD = 0.02
# steer 1.0 对应的角速度贡献 (减号 = 实车标定回正方向)
CORR_WEIGHT = 0.3

# ---------------- 巡线滤波 (前视推理线程侧) ----------------
# 一阶低通 EMA 平滑系数 0~1: 越大越跟手(含噪声), 越小越平滑(延迟大)
LANE_EMA = 0.9
# 单次推理异常超过该秒数则按"无误差直行"处理; 丢帧时更稳
LANE_TIMEOUT = 0.1

# ---------------- 前进速度 ----------------
# 前进速度独立恒定(不随误差降速), 与转弯速度互不耦合
V_FORWARD = 0.6  # 默认前进速度 (m/s): lane_base(speed) 未显式传入时兜底; 触发配置的 speed 会覆盖它
