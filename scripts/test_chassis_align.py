#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘对齐「前进/后退(cx)方向」测试 — 两种臂姿下的方向映射验证.

背景:
    chassis_align 里画面横向误差 cx_err = cx - px 驱动车前后速度
    vx = sign_y * kp_y * cx_err (sign_y = sign[1] = 前后符号)。
    侧视相机朝向随大臂转向切换, 同样"目标在画面左"对应相反的车体运动:

      情况1 竖拍 (大臂≈-93, 末端≈-70):
            目标在画面左 → 车前进, 目标在画面右 → 车后退   → sign_y = +1
      情况2 横拍 (大臂≈+93, 末端≈-20):
            目标在画面左 → 车后退, 目标在画面右 → 车前进   → sign_y = -1

用法:
    python scripts/test_chassis_align.py --check            # 默认: 离线逻辑断言(无需实车)
    python scripts/test_chassis_align.py --live             # 实车方向探针(逐情况验证符号, 不定居中)
    python scripts/test_chassis_align.py --live --case 1    # 只跑竖拍
    python scripts/test_chassis_align.py --live --label water
    python scripts/test_chassis_align.py --align --label cylinder_set --case 2   # 真对齐闭环: 把目标居中
    python scripts/test_chassis_align.py --align --case 2 --cy 0.2 --sign-x -1   # 指定横向符号/期望cy
    python scripts/test_chassis_align.py --align --case 2 --kp 0.15,0.08 --deadband 0.03 --v-min 0.005  # 调PID再对齐
    python scripts/test_chassis_align.py --ik               # 逆解诊断(方案3): 纯运动快照+轮级真值表→修正矩阵
    python scripts/test_chassis_align.py --ik --truth 'FL:+1,FR:-1,RL:-1,RR:+1'  # 预填真值表跳过提问
    python scripts/test_chassis_align.py --align --label cylinder_set --case 2  --prefer-left       --kp 0.15,0.08 --deadband 0.03 --hold 6 --v-max 0.12 --v-min 0.005 --no-decouple

--check 只验纯逻辑(符号表 + 模拟 vx 方向), 跑通后把 resolve_fwd_sign / fwd_vx
移入 tasks/tools/motion/locate.py, 让 chassis_align 按大臂档位自动定 sign_y。
(默认用本文件副本; 移入后加 --src locate 测 locate 真身 —— 惰性导入,
  避免在无硬件/PC 上 import tasks 拉起硬件链挂起。)
--live 才是验证"符号表是否符合实车物理方向"的关键: 按假设符号先算收敛方向
(vx_cmd = sign_y*kp*(cx-px) 减误差那侧), 先朝收敛方向开 —— 目标应朝期望点靠拢;
再反方向开 —— 应远离。两段都符合 → PASS; 收敛段反了 → FAIL, 说明 resolve_fwd_sign
的档位表要翻。目标出框/消失记 INCONCLUSIVE(常是初始目标太贴边、探针方向把目标
顶出视野, 移车让目标居中后重跑), 不再误报"方向反了"。
--live 只验证方向(小幅挪动看目标靠近/远离), 不做居中。要看真对齐闭环用 --align:
摆臂姿 → 自动横向探针判 sign_x(画面纵向误差→车横向的符号) → 调 car.chassis_align
真正把目标居中 → 报告收敛/残差。

⚠️ 相机身份(已核实): cap_side = cam4 = 机械臂相机 —— config_car.yml
camera.side=4, udev 注释明写"cam4 = 机械臂相机", 装在臂上随大臂转
(推流标签即 cam2, realtime.py)。所以大臂 -93(竖拍)/+93(横拍)翻转
画面横向→车前后映射是物理必然, 按大臂角度分档成立; cy→横移符号
两档也随臂一起翻转, 同样要分档。--live 前进/后退探针当场裁决符号表。
急停: Ctrl+C
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ================================================================
# 方向解析逻辑 (验证通过后移入 tasks/tools/motion/locate.py)
# ================================================================
def resolve_fwd_sign(arm_angle, calibrated=None):
    """大臂角度档位 → 画面横向误差(cx)驱动车前后(vx)的符号.

    竖拍(大臂≈-93): 目标在画面左 → 车前进, 右 → 车后退 → +1
    横拍(大臂≈+93): 目标在画面左 → 车后退, 右 → 车前进 → -1
    中间区(|arm|<45°)方向无定论 → 返回 0.0(不自动驱动前后, 需人工给 sign)。
    calibrated: 现场人工确认过的符号, 非 None 时优先返回。
    """
    if calibrated is not None:
        return float(calibrated)
    if arm_angle <= -45.0:
        return 1.0
    if arm_angle >= 45.0:
        return -1.0
    return 0.0


def fwd_vx(arm_angle, cx_err, kp_y=0.22, sign_y=None):
    """按大臂档位符号算车前后速度 vx = sign_y * kp_y * cx_err (镜像 chassis_align)."""
    if sign_y is None:
        sign_y = resolve_fwd_sign(arm_angle)
    return sign_y * kp_y * cx_err


# 默认用本文件副本; --check --src locate 时在 Jetson 上导入 locate.py 真身验证
# (惰性导入: 直接 import tasks 会拉起硬件链, 在无硬件/PC 上会挂起而非报错)


# ================================================================
# 离线逻辑检查 (--check, 默认)
# ================================================================
def run_check(src="local"):
    ok = True
    rs = resolve_fwd_sign
    fv = fwd_vx
    if src == "locate":
        from tasks.tools.motion.locate import (  # 惰性: 仅在 Jetson 上显式要求时
            resolve_fwd_sign as _rf,
            fwd_vx as _fv,
        )
        rs, fv = _rf, _fv

    def expect(name, cond):
        nonlocal ok
        if not cond:
            ok = False
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")

    print(f"逻辑来源: {src}\n")
    print("== 符号表 ==")
    expect("竖拍 大臂-93 → sign_y=+1", rs(-93) == 1.0)
    expect("横拍 大臂+93 → sign_y=-1", rs(+93) == -1.0)
    expect("中间区 大臂0 → 0 (不驱动前后)", rs(0.0) == 0.0)
    expect("calibrated 现场覆盖优先", rs(-93, calibrated=-1.0) == -1.0)

    print("== 情况1 竖拍: 目标左→前进, 目标右→后退 ==")
    expect("目标在左 (cx_err=+0.05) → vx>0 前进", fv(-93, +0.05) > 0)
    expect("目标在右 (cx_err=-0.05) → vx<0 后退", fv(-93, -0.05) < 0)

    print("== 情况2 横拍: 目标左→后退, 目标右→前进 ==")
    expect("目标在左 (cx_err=+0.05) → vx<0 后退", fv(+93, +0.05) < 0)
    expect("目标在右 (cx_err=-0.05) → vx>0 前进", fv(+93, -0.05) > 0)

    print("== 对称性 / 量纲 ==")
    expect("vx 与误差线性: fwd_vx(-err) = -fwd_vx(+err)",
           abs(fv(-93, -0.05) + fv(-93, +0.05)) < 1e-12)
    expect("vx 随 kp_y 线性缩放",
           abs(fv(-93, 0.05, kp_y=0.44) - 2 * fv(-93, 0.05, kp_y=0.22)) < 1e-12)

    print(f"\n结果: {'全部通过 ✓' if ok else '存在失败 ✗'}")
    return ok


# ================================================================
# 实车方向探针 (--live)
# ================================================================
# 竖拍/横拍的臂姿(与 test_arm_pose.py 一致); sign_y 由 resolve_fwd_sign 按大臂档位给出,
# 探针按"假设符号下的收敛方向"先开收敛段、再反方向开发散段。
ARM_CASES = {
    1: ("竖拍", dict(x=-0.20, y=-0.02, arm=-93, hand=-70)),
    2: ("横拍", dict(x=-0.22, y=-0.15, arm=+93, hand=-20)),
}
PROBE_V = 0.08   # 探针速度 (m/s); 0.8s ≈ 6.4cm, 安全小幅
PROBE_DUR = 0.8


def _read_pxy(car, label, wait=5.0, max_age=0.5):
    """等缓存里出现该 label, 返回归一化中心 (nx, ny); 超时/消失返回 (None, None)."""
    end = time.time() + wait
    while time.time() < end:
        for d in car.get_realtime_detections(max_age=max_age):
            if d[2] == label:
                return d[4], d[5]
        time.sleep(0.05)
    return None, None


def _read_px(car, label, wait=5.0, max_age=0.5):
    xy = _read_pxy(car, label, wait=wait, max_age=max_age)
    return xy[0]


def _wait_x(car, target, tol=0.003, timeout=8.0):
    """轮询等待 X 轴到达 target(容差 tol 米). 到位返回 True, 超时返回 False."""
    end = time.time() + timeout
    while time.time() < end:
        if abs(car.arm.x_get_position() - target) < tol:
            return True
        time.sleep(0.1)
    return False


def _move_x_sure(car, target, tol=0.003, retries=3):
    """确保 X 轴到位: 发 goto_position → 轮询确认 → 未到位自动重试, 重试前先撞墙复位重标 0 点.

    - goto_position 是绝对目标, 重发会从当前位置继续走到 target;
    - reset_x 把 X 撞到右端墙、重标 0 点, 修复"0 点被污染导致的目标位置偏差";
    - 每次都打印当前 X 位置供肉眼核对: 若打印的 now 与滑块实际位置明显不符,
      说明编码器换算系数(arm_cfg.yaml 的 perimeter)要校准, 软件重试救不了, 请报给我.
    返回 True=确认到位 / False=重试耗尽仍未到位.
    """
    for attempt in range(1, retries + 1):
        car.arm.goto_position(x=target, speed=[0.25, 0.05])
        if _wait_x(car, target, tol):
            print(f"  [X 到位] target={target:+.3f} now={car.arm.x_get_position():+.3f}")
            return True
        print(f"  [X 未到位] 第{attempt}次: target={target:+.3f} "
              f"now={car.arm.x_get_position():+.3f}")
        if attempt < retries:
            if attempt == retries - 1:      # 最后一次重试前, 先撞墙复位重标 0 点
                print("    → X 撞墙复位(重标 0 点)后重试...")
                car.arm.reset_x()
                time.sleep(0.5)
            else:
                print("    → 直接重走一次...")
    print(f"  [FAIL] X 反复不到位 target={target:+.3f} now={car.arm.x_get_position():+.3f} "
          f"→ 需查机械/标定(perimeter)")
    return False


def _burst(car, vx):
    """按 vx(m/s) 驱动一小段再停下, 等目标 px 稳定."""
    car.set_velocity(vx, 0, 0)
    time.sleep(PROBE_DUR)
    car.set_velocity(0, 0, 0)
    time.sleep(0.3)


def _run_case(car, label, cx, case_no):
    """单档实车方向裁决: 按假设符号算出收敛方向, 先收敛段后发散段.

    返回 True=PASS / False=FAIL / None=未裁决(目标出框/消失或中间区无符号)。
    """
    name, cfg = ARM_CASES[case_no]
    print(f"\n===== 情况{case_no} {name}: 大臂{cfg['arm']} 末端{cfg['hand']} =====")
    _move_x_sure(car, cfg["x"])                    # 先确保 X 到位
    car.arm.set_arm_pose(cfg["x"], cfg["y"], cfg["arm"], cfg["hand"])
    time.sleep(0.8)

    px = _read_px(car, label, wait=6.0)
    if px is None:
        print(f"  [INCONCLUSIVE] 6s 内未见目标 {label}, 跳过该情况")
        return None
    err = cx - px
    side = "右" if err < 0 else "左"
    print(f"  目标在期望点(cx={cx:.3f})之{side}侧: px={px:.3f}")

    # 假设符号: vx_cmd = sign_y*kp*(cx-px) → 减误差的那侧 = 收敛方向
    sign_y = resolve_fwd_sign(cfg["arm"])
    if sign_y == 0:
        print("  [INCONCLUSIVE] 大臂在中间区(|arm|<45°), 无自动符号, 跳过")
        return None
    conv_dir = 1.0 if sign_y * err > 0 else -1.0   # +1=前进, -1=后退
    print(f"  假设 sign_y={sign_y:+.0f} → 收敛方向={'前进' if conv_dir > 0 else '后退'}")

    # ① 收敛段: 朝收敛方向开, 目标应靠拢期望点
    _burst(car, +PROBE_V * conv_dir)
    p1 = _read_px(car, label, wait=3.0)
    if p1 is None:
        print("  [INCONCLUSIVE] 收敛探针期间目标消失(目标本贴边, 或假设方向错把它顶出视野)")
        print("              → 移车让目标居中后重跑; 若重跑仍出框, 才考虑翻 sign_y")
        return None
    conv = abs(cx - p1) < abs(cx - px)
    print(f"  收敛 {PROBE_DUR}s: px {px:.3f} → {p1:.3f} (Δ{p1 - px:+.3f}) → "
          f"{'靠近' if conv else '远离'}期望点")
    print(f"  [{'PASS' if conv else 'FAIL'}] 收敛方向 {'符合假设' if conv else '反了 → 需翻 sign_y'}")

    # ② 发散段: 反方向开, 目标应远离期望点
    _burst(car, -PROBE_V * conv_dir)
    p2 = _read_px(car, label, wait=3.0)
    if p2 is None:
        print("  [INCONCLUSIVE] 发散探针期间目标消失(不影响收敛段结论)")
        return conv
    div = abs(cx - p2) > abs(cx - p1)
    print(f"  发散 {PROBE_DUR}s: px {p1:.3f} → {p2:.3f} (Δ{p2 - p1:+.3f}) → "
          f"{'远离' if div else '靠近'}期望点")
    print(f"  [{'PASS' if div else 'FAIL'}] 发散方向")
    return conv and div


# ================================================================
# 真对齐闭环 (--align): 摆臂姿 → 判双轴符号 → car.chassis_align 实际居中
# ================================================================
def _probe_lateral(car, label, cy, wait=5.0):
    """横向探针: 自动判 sign_x(画面纵向误差 cy_err → 车横向 vy 的符号).

    假设 sign_x=+1: vy_cmd = +kp*cy_err → 试探方向 = sign(cy_err)。
    往该方向开, py 若靠近 cy → sign_x=+1, 远离 → -1. 目标消失返回 None."""
    _, py = _read_pxy(car, label, wait=wait)
    if py is None:
        return None
    cy_err = cy - py
    if abs(cy_err) < 1e-6:
        print(f"  横向探针: py≈cy={cy:.3f} 已在期望, 取 sign_x=+1")
        return 1.0
    test_dir = 1.0 if cy_err > 0 else -1.0
    car.set_velocity(0.0, PROBE_V * test_dir, 0.0)
    time.sleep(PROBE_DUR)
    car.set_velocity(0.0, 0.0, 0.0)
    time.sleep(0.3)
    _, p1 = _read_pxy(car, label, wait=3.0)
    if p1 is None:
        return None
    conv = abs(cy - p1) < abs(cy - py)
    print(f"  横向探针: py {py:.3f} → {p1:.3f} (Δ{p1 - py:+.3f}) → "
          f"{'靠近' if conv else '远离'}cy={cy:.3f} → sign_x={'+1' if conv else '-1'}")
    return 1.0 if conv else -1.0


def _align_case(car, label, cx, cy, case_no, sign_x_override=None, timeout=8.0,
                kp=(0.15, 0.08), deadband=0.03, hold=6, v_max=0.12, v_min=0.005,
                decouple=True, prefer_left=False, prefer_right=False):
    """真对齐闭环: 摆臂姿 → 判双轴符号 → 调 car.chassis_align 实际居中目标.

    返回 True=收敛 / False=超时未收敛 / None=未裁决(目标消失或中间区无符号)。
    """
    name, cfg = ARM_CASES[case_no]
    print(f"\n===== 情况{case_no} {name}: 真对齐闭环 =====")
    _move_x_sure(car, cfg["x"])                    # 先确保 X 到位
    car.arm.set_arm_pose(cfg["x"], cfg["y"], cfg["arm"], cfg["hand"])
    time.sleep(0.8)

    sign_y = resolve_fwd_sign(cfg["arm"])
    if sign_y == 0:
        print("  [INCONCLUSIVE] 大臂在中间区(|arm|<45°), 无自动符号, 跳过")
        return None
    if sign_x_override is not None:
        sign_x = float(sign_x_override)
        print(f"  横向符号 sign_x 用覆盖值 {sign_x:+.0f}")
    else:
        sign_x = _probe_lateral(car, label, cy)
        if sign_x is None:
            print("  [INCONCLUSIVE] 横向探针期间目标消失, 无法定 sign_x; 可用 --sign-x 手填")
            return None
    print(f"  符号 sign = (sign_x {sign_x:+.0f}, sign_y {sign_y:+.0f})  "
          f"kp={kp} deadband={deadband} hold={hold} v_max={v_max} v_min={v_min} "
          f"decouple={decouple} prefer_l={prefer_left} prefer_r={prefer_right}")

    t0 = time.monotonic()
    ok = car.chassis_align(label, cx=cx, cy=cy, kp=kp, sign=(sign_x, sign_y),
                           deadband=deadband, hold=hold, v_max=v_max, v_min=v_min,
                           decouple_xy=decouple, prefer_left=prefer_left,
                           prefer_right=prefer_right, timeout=timeout)
    dt = time.monotonic() - t0
    px, py = _read_pxy(car, label, wait=2.0)
    if px is None:
        resid = "目标当前不可见"
    else:
        resid = f"残差 cx={px - cx:+.3f} cy={py - cy:+.3f} (px={px:.3f} py={py:.3f})"
    print(f"  [{'PASS' if ok else 'FAIL'}] 对齐 {dt:.1f}s: {resid}")
    return ok


def _run_align(args):
    """真对齐闭环入口: 逐档摆臂姿 → 自动判 sign_x → car.chassis_align 居中 → 报告."""
    from tasks.tools import create_car
    car = create_car()          # reset=True: 臂复位到已知位
    try:
        cases = (args.case,) if args.case else (1, 2)
        all_ok, inconcl = True, False
        for c in cases:
            r = _align_case(car, args.label, args.cx, args.cy, c,
                            sign_x_override=args.sign_x, timeout=args.align_timeout,
                            kp=args.kp, deadband=args.deadband, hold=args.hold,
                            v_max=args.v_max, v_min=args.v_min,
                            decouple=not args.no_decouple,
                            prefer_left=args.prefer_left, prefer_right=args.prefer_right)
            if r is False:
                all_ok = False
            elif r is None:
                inconcl = True
            print("  稍候 1.5s...")
            time.sleep(1.5)
        if inconcl:
            print("\n结果: 有档未裁决(目标消失/中间区) → 移车或 --sign-x 手填后重跑")
            return 1
        print(f"\n结果: {'对齐全部到位 ✓' if all_ok else '存在未到位 ✗'}")
        return 0 if all_ok else 1
    except KeyboardInterrupt:
        print("\n急停")
        return 1
    finally:
        car.stop()
        car.close()


# ================================================================
# 逆解诊断 (--ik) : 方案3 — 轮级真值表 → 修正后的逆解/正解矩阵
#   (只诊断 + 打印可粘贴矩阵, 不写盘; 贴进 mecanum.py 后重跑复测)
# ================================================================
IK_WHEELS = ("FL", "FR", "RL", "RR")   # 物理轮位(以车头为前)
# 底盘 docstring 端口定义: port_list [1,2,3,4] → [FR,FL,RL,RR] (仅默认提示, 以观察为准)
IK_PORT_HINT = {0: "FR(右前)", 1: "FL(左前)", 2: "RL(左后)", 3: "RR(右后)"}
# 标准麦轮物理模式(按物理轮 FL/FR/RL/RR), 值 = 该轮在"前进向"上的速度系数
_IK_VX = {"FL": 1.0, "FR": 1.0, "RL": 1.0, "RR": 1.0}    # 前进: 四轮同向前
_IK_VY = {"FL": -1.0, "FR": 1.0, "RL": 1.0, "RR": -1.0}  # +vy(右横移): 对角对反向
_IK_Z = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}   # +z: 左右侧反向(CCW)


def run_ik(args):
    """逆解诊断: ①当前纯运动快照 ②轮级真值表 ③推导修正矩阵(打印) ④期望行为对照."""
    import numpy as np
    from tasks.tools import create_car

    car = create_car(reset=False)   # 不动臂/不清里程计来源; 只用底盘轮
    try:
        cur = np.array(car.chassis.vehicle_to_wheel_matrix, dtype=float)
        tan_val = float(abs(cur[1][0]))
        c_val = float(abs(cur[2][0]))

        if not args.no_motion:
            print("⚠️ 请架空四轮或置于开阔场地运行, 车会短暂移动/旋转\n")
            print("===== 1) 当前矩阵的纯运动快照 (读里程计) =====")
            for name, cmd in [("前进  vx=+0.12", (0.12, 0, 0)),
                              ("右横移 vy=+0.12", (0, 0.12, 0)),
                              ("原地转  z=+0.40", (0, 0, 0.40))]:
                car.reset_position()
                time.sleep(0.1)
                car.set_velocity(*cmd)
                time.sleep(0.6)
                car.set_velocity(0, 0, 0)
                time.sleep(0.3)
                x, y, t = car.get_odometry()
                print(f"  {name}: dx={x:+.3f} dy={y:+.3f} dth={t:+.3f}")
            print("  (判读: 前进应 dx 动; 右横移应 dy 动且 dth≈0; 原地转应 dth 动)")
            print("  若横移出现 dth 明显而 dy 不动 → 逆解要修\n")

        print("===== 2) 轮级点动真值表 (观察物理轮) =====")
        print("  以车头为前: FL=左前 FR=右前 RL=左后 RR=右后")
        print("  底盘 docstring 端口定义: 索引0=FR 1=FL 2=RL 3=RR (仅默认, 以你观察为准)")
        truth = {}
        if args.truth:
            for i, tok in enumerate(args.truth.split(",")):
                w, d = tok.strip().split(":")
                truth[i] = (w.strip().upper(), float(d))
            print(f"  使用 --truth 预设: {truth}")
        else:
            for i in range(4):
                v = [0.0, 0.0, 0.0, 0.0]
                v[i] = 0.2
                print(f"\n  点动索引{i} (+0.2, 2s)...", end="", flush=True)
                car.wheels_chassis.set_linear(v)
                time.sleep(2.0)
                car.wheels_chassis.set_linear([0.0, 0.0, 0.0, 0.0])
                time.sleep(0.5)
                print(" 观察完成.")
                w = input(f"  哪只物理轮在转? (回车默认 {IK_PORT_HINT[i]}) [FL/FR/RL/RR]: ").strip().upper() \
                    or IK_PORT_HINT[i].split("(")[0]
                while w not in IK_WHEELS:
                    w = input(f"    '{w}' 无效, 输入 FL/FR/RL/RR: ").strip().upper()
                d = input(f"  该轮 +speed 时朝哪个方向转? [F 前/B 后]: ").strip().upper()
                d = 1.0 if d.startswith("F") else -1.0
                truth[i] = (w, d)
                print(f"  ✓ 索引{i} → {w}, d={d:+.0f}")

        print("\n===== 3) 由真值表推导修正矩阵 =====")
        M = np.zeros((3, 4))
        W = np.zeros((4, 3))
        patterns = (_IK_VX, _IK_VY, _IK_Z)
        cf = (1.0, tan_val, c_val)
        for row, pat in enumerate(patterns):
            for i in range(4):
                w, d = truth[i]
                M[row][i] = pat[w] * d * cf[row]
        for i in range(4):
            for row in range(3):
                # W 是 M 的精确逆: W[i][row] = M[row][i] / (4 * cf[row]**2)
                # (除以行范数平方; 只除 4*cf 会 M@W=diag(cf)≠I, 里程计 vy/z 恢复不准)
                W[i][row] = M[row][i] / (4.0 * cf[row] ** 2)

        print(f"  tan_roller≈{tan_val:.4f}  wheel_constant≈{c_val:.4f}")
        same_vxvy = np.allclose(M[:2], cur[:2], atol=1e-9)
        z_same = np.sign(M[2][0]) == np.sign(cur[2][0]) if M[2][0] and cur[2][0] else True
        if same_vxvy and z_same:
            print("  ⚠️ 推导矩阵与当前矩阵一致 → 逆解没错, 横移问题不在这, 别改矩阵!")
        elif same_vxvy and not z_same:
            print("  ⚠️ vx/vy 行与当前一致; 仅 z(旋转)行整体符号相反 = 旋转方向约定差,")
            print("     若现车旋转方向是对的, z 行保持当前符号即可, 不用贴下面矩阵")
        else:
            print("  ⚠️ 推导矩阵与当前矩阵不同 → 下面两段可贴进 mecanum.py 替换")

        print("\n  # vehicle_to_wheel_matrix (3x4, 行=车vx/vy/z, 列=命令索引0..3)")
        print("  M = np.array([")
        for row in range(3):
            print("      [{}],".format(", ".join(f"{M[row][j]:+.4f}" for j in range(4))))
        print("  ])")
        print("\n  # wheel_to_vehicle_matrix (4x3, 行=命令索引, 列=车vx/vy/z)")
        print("  W = np.array([")
        for i in range(4):
            print("      [{}],".format(", ".join(f"{W[i][row]:+.6f}" for row in range(3))))
        print("  ])")

        print("\n===== 4) 推导矩阵下的期望物理行为 (对照实车) =====")
        for name, cmd in [("前进", (0.10, 0, 0)), ("右横移", (0, 0.10, 0)), ("原地转", (0, 0, 0.30))]:
            wv = np.array(cmd) @ M
            descs = []
            for i in range(4):
                w, d = truth[i]
                phys = wv[i] * d
                descs.append(f"{w}:{'前' if phys > 1e-9 else ('后' if phys < -1e-9 else '停')}")
            print(f"  {name}: " + " ".join(descs))

        chk = np.array([0.12, -0.08, 0.05]) @ M @ W
        print("\n  [自检] (vx,vy,z)@M@W ≈", np.round(chk, 6), "(应≈原命令)")
        print("  → 若需要替换, 贴进 smartcar/whalesbot/vehicle/driver/mecanum.py 后")
        print("    重跑: python scripts/test_chassis_align.py --ik --truth '<同上>' 复测快照")
        return 0
    except KeyboardInterrupt:
        print("\n急停")
        return 1
    finally:
        car.stop()
        car.close()


def main():
    # Windows 控制台默认 GBK, 打不出 ✓/✗/中文对齐; 强制 UTF-8 输出兜底(Jetson 上本就是 UTF-8)
    for _s in (sys.stdout, sys.stderr):
        try:
            _s.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description="底盘对齐前进/后退(cx)方向测试")
    ap.add_argument("--check", action="store_true", help="离线逻辑断言(默认行为)")
    ap.add_argument("--src", choices=("local", "locate"), default="local",
                    help="--check 时被测逻辑来源: local=本文件副本(默认) / "
                         "locate=locate.py 真身(需已移入且在有硬件链的 Jetson 上跑)")
    ap.add_argument("--ik", action="store_true",
                    help="逆解诊断(方案3): 纯运动快照+轮级真值表→推导修正矩阵(不写盘)")
    ap.add_argument("--truth", default=None,
                    help="--ik: 预填真值表(索引序), 如 'FL:+1,FR:-1,RL:-1,RR:+1', 跳过点动提问")
    ap.add_argument("--no-motion", action="store_true", help="--ik: 跳过纯运动快照")
    ap.add_argument("--live", action="store_true", help="实车方向探针(只验符号, 不定居中)")
    ap.add_argument("--align", action="store_true",
                    help="真对齐闭环: 摆臂姿→自动判 sign_x→调 car.chassis_align 实际居中")
    ap.add_argument("--case", type=int, choices=(1, 2),
                    help="--live/--align: 1=只竖拍 2=只横拍, 缺省两档都跑")
    ap.add_argument("--label", default="water", help="--live/--align: 对齐目标类别(默认 water)")
    ap.add_argument("--cx", type=float, default=0.142, help="期望点 cx(默认 0.142)")
    ap.add_argument("--cy", type=float, default=0.0, help="--align: 期望点 cy(默认 0)")
    ap.add_argument("--sign-x", type=float, default=None,
                    help="--align: 横向符号 sign_x 覆盖(默认自动探针判定)")
    ap.add_argument("--align-timeout", type=float, default=8.0,
                    help="--align: 对齐闭环超时秒数(默认 8)")
    ap.add_argument("--kp", type=str, default="0.15,0.08",
                    help="--align: PID 增益 (kp_x左右,kp_y前后) 逗号分隔, 默认 0.15,0.08")
    ap.add_argument("--deadband", type=float, default=0.03,
                    help="--align: 收敛死区(两轴误差<该值进死区), 默认 0.03")
    ap.add_argument("--hold", type=int, default=6,
                    help="--align: 进死区需连续保持的帧数(20Hz), 默认 6")
    ap.add_argument("--v-max", type=float, default=0.12,
                    help="--align: 底盘速度上限(m/s), 默认 0.12")
    ap.add_argument("--v-min", type=float, default=0.005,
                    help="--align: 输出死区(|v|<该值置0), 默认 0.005; 调大防抖/调小防静差")
    ap.add_argument("--no-decouple", action="store_true",
                    help="--align: 关闭 decouple_xy 单轴防滑(两轴同时驱动), 默认开")
    ap.add_argument("--prefer-left", action="store_true",
                    help="--align: 多目标时锁定画面最左侧(px最小), 默认否")
    ap.add_argument("--prefer-right", action="store_true",
                    help="--align: 多目标时锁定画面最右侧(px最大); 同 prefer-left 时左优先")
    args = ap.parse_args()

    try:
        kp = tuple(float(t) for t in args.kp.split(","))
        if len(kp) != 2:
            raise ValueError
    except ValueError:
        ap.error("--kp 需为 'kx,ky' 两个逗号分隔数字, 如 --kp 0.10,0.04")
    args.kp = kp

    if args.ik:
        return run_ik(args)
    if args.align:
        return _run_align(args)
    if not args.live:
        return 0 if run_check(src=args.src) else 1

    from tasks.tools import create_car
    car = create_car()          # reset=True: 臂复位到已知位, 便于摆到测试姿态
    try:
        cases = (args.case,) if args.case else (1, 2)
        all_ok, inconcl = True, False
        for c in cases:
            r = _run_case(car, args.label, args.cx, c)
            if r is False:
                all_ok = False
            elif r is None:
                inconcl = True
            print("  稍候 1.5s...")
            time.sleep(1.5)
        if inconcl:
            print("\n结果: 有情况未裁决(目标出框/消失) → 移车让目标居中后重跑")
            return 1
        print(f"\n结果: {'两个方向映射全部符合预期 ✓' if all_ok else '存在方向不符 ✗ → 需翻 resolve_fwd_sign'}")
        return 0 if all_ok else 1
    except KeyboardInterrupt:
        print("\n急停")
        return 1
    finally:
        car.stop()
        car.close()


if __name__ == "__main__":
    sys.exit(main())
