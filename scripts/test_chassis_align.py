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
    python scripts/test_chassis_align.py --live             # 实车方向探针(逐情况验证)
    python scripts/test_chassis_align.py --live --case 1    # 只跑竖拍
    python scripts/test_chassis_align.py --live --label water
    python scripts/test_chassis_align.py --ik               # 逆解诊断(方案3): 纯运动快照+轮级真值表→修正矩阵
    python scripts/test_chassis_align.py --ik --truth 'FL:+1,FR:-1,RL:-1,RR:+1'  # 预填真值表跳过提问

--check 只验纯逻辑(符号表 + 模拟 vx 方向), 跑通后把 resolve_fwd_sign / fwd_vx
移入 tasks/tools/motion/locate.py, 让 chassis_align 按大臂档位自动定 sign_y。
(默认用本文件副本; 移入后加 --src locate 测 locate 真身 —— 惰性导入,
  避免在无硬件/PC 上 import tasks 拉起硬件链挂起。)
--live 才是验证"符号表是否符合实车物理方向"的关键: 前进/后退探针看目标 px 是否
按预期朝画面中心靠拢(竖拍) / 远离(横拍)。符合 → PASS, 反了 → FAIL,
说明 resolve_fwd_sign 的档位表要翻。

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
# 竖拍/横拍的臂姿, 与"前进应使目标靠近还是远离期望点"
#   expect_toward=True  = 正确符号下前进会让目标朝画面中心靠拢
#   expect_toward=False = 正确符号下前进会让目标远离(该档该后退)
ARM_CASES = {
    1: ("竖拍", dict(x=-0.20, y=-0.02, arm=-93, hand=-70, expect_toward=True)),
    2: ("横拍", dict(x=-0.22, y=-0.15, arm=+93, hand=-20, expect_toward=False)),
}
PROBE_V = 0.08   # 探针速度 (m/s); 0.8s ≈ 6.4cm, 安全小幅
PROBE_DUR = 0.8


def _read_px(car, label, wait=5.0, max_age=0.5):
    """等缓存里出现该 label, 返回归一化 cx; 超时/目标消失返回 None."""
    end = time.time() + wait
    while time.time() < end:
        for d in car.get_realtime_detections(max_age=max_age):
            if d[2] == label:
                return d[4]
        time.sleep(0.05)
    return None


def _burst(car, vx):
    """按 vx(m/s) 驱动一小段再停下, 等目标 px 稳定."""
    car.set_velocity(vx, 0, 0)
    time.sleep(PROBE_DUR)
    car.set_velocity(0, 0, 0)
    time.sleep(0.3)


def _run_case(car, label, cx, case_no):
    name, cfg = ARM_CASES[case_no]
    print(f"\n===== 情况{case_no} {name}: 大臂{cfg['arm']} 末端{cfg['hand']} =====")
    car.arm.set_arm_pose(cfg["x"], cfg["y"], cfg["arm"], cfg["hand"])
    time.sleep(0.8)

    px = _read_px(car, label, wait=6.0)
    if px is None:
        print(f"  [FAIL] 6s 内未见目标 {label}, 跳过该情况")
        return False
    side = "左" if px < cx else "右"
    print(f"  目标在期望点(cx={cx:.3f})之{side}侧: px={px:.3f}")

    # 前进探针: 正确符号下, 竖拍应使目标靠近中心, 横拍应使目标远离中心
    _burst(car, +PROBE_V)
    p1 = _read_px(car, label, wait=3.0)
    if p1 is None:
        print("  [FAIL] 前进探针期间目标消失")
        return False
    f_toward = abs(cx - p1) < abs(cx - px)
    ok_f = (f_toward == cfg["expect_toward"]) and abs(p1 - px) > 1e-4
    print(f"  前进 {PROBE_DUR}s: px {px:.3f} → {p1:.3f} (Δ{p1 - px:+.3f}) "
          f"→ {'靠近' if f_toward else '远离'}期望点 (预期{'靠近' if cfg['expect_toward'] else '远离'})")
    print(f"  [{'PASS' if ok_f else 'FAIL'}] 前进方向 {'符合' if ok_f else '反了'}")

    # 后退探针: 应与前进相反(回到原点附近)
    _burst(car, -PROBE_V)
    p2 = _read_px(car, label, wait=3.0)
    if p2 is None:
        print("  [FAIL] 后退探针期间目标消失")
        return False
    b_toward = abs(cx - p2) < abs(cx - p1)
    ok_b = (b_toward != f_toward) and abs(p2 - p1) > 1e-4
    print(f"  后退 {PROBE_DUR}s: px {p1:.3f} → {p2:.3f} (Δ{p2 - p1:+.3f}) "
          f"→ {'远离' if b_toward else '靠近'}期望点, 与前进{'相反 ✓' if b_toward != f_toward else '相同 ✗'}")
    print(f"  [{'PASS' if ok_b else 'FAIL'}] 后退方向")

    return ok_f and ok_b


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
    ap.add_argument("--live", action="store_true", help="实车方向探针")
    ap.add_argument("--case", type=int, choices=(1, 2),
                    help="--live: 1=只竖拍 2=只横拍, 缺省两档都跑")
    ap.add_argument("--label", default="water", help="--live: 对齐目标类别(默认 water)")
    ap.add_argument("--cx", type=float, default=0.142, help="--live: 期望点 cx(默认 0.142)")
    args = ap.parse_args()

    if args.ik:
        return run_ik(args)
    if not args.live:
        return 0 if run_check(src=args.src) else 1

    from tasks.tools import create_car
    car = create_car()          # reset=True: 臂复位到已知位, 便于摆到测试姿态
    try:
        cases = (args.case,) if args.case else (1, 2)
        all_ok = True
        for c in cases:
            if not _run_case(car, args.label, args.cx, c):
                all_ok = False
            print("  稍候 1.5s...")
            time.sleep(1.5)
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
