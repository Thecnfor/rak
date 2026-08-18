# -*- coding: utf-8 -*-
"""全局坐标层: 在轮式里程计之上维护一个"场地坐标系"(不随 reset_position 丢失).

背景:
    底盘 Odometry 提供 [x, y, theta](米/弧度), 但 reset_position 会把它清零,
    于是"车在场地哪里"这个信息在全流程中反复丢失(启动清零 / sorting 后清零)。
    本模块把"当前里程计坐标系 -> 场地坐标系"的 SE(2) 变换单独存一份:
        global = R(theta_off) @ odom_xy + (tx, ty);  theta_g = theta_o + theta_off
    里程计随便清零, 变换跟着重锚, 全局轨迹保持连续。

用法(经 MyCar 封装, 任务层直接用):
    car.set_field_origin(0, 0, 0)        # 车摆在场地原点朝 +X 时锚定
    car.get_global_pose()                # -> [x, y, theta] 场地系
    car.go_to_global_pose([1.2, 0.5, math.pi / 2])   # 按全局坐标闭环导航

注意: 无 IMU, theta 靠轮式积分, 麦轮打滑会累积漂移; 长途后建议配合
视觉对齐(chassis_align)或重新 set_field_origin 修正。
"""
import math
import threading
from typing import Callable, List, Sequence, Union

Pose = Union[Sequence[float], List[float]]


def wrap_angle(a: float) -> float:
    """角度归一化到 [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class GlobalPose:
    """场地坐标系维护器: 存"里程计系 -> 场地系"的 SE(2) 变换, reset 安全."""

    def __init__(self, odom_getter: Callable[[], Pose] = lambda: [0.0, 0.0, 0.0]):
        """
        参数:
            odom_getter: 返回当前里程计位姿 [x, y, theta] 的回调(传 car.get_odometry).
                只在 anchor(不传 odom_pose)时被调用, 平时可传入 None 之外的任何可调用。
        """
        self._get_odom = odom_getter
        self._lock = threading.Lock()
        # 初始: 场地系与里程计系重合(开机时车在哪, 场地原点就在哪)
        self._tx = 0.0
        self._ty = 0.0
        self._theta_off = 0.0

    # ------------------------------------------------------------------
    # 变换
    # ------------------------------------------------------------------
    def to_global(self, odom_pose: Pose) -> List[float]:
        """里程计位姿 -> 场地系位姿 [x, y, theta]."""
        x, y, th = float(odom_pose[0]), float(odom_pose[1]), float(odom_pose[2])
        with self._lock:
            tx, ty, off = self._tx, self._ty, self._theta_off
        c, s = math.cos(off), math.sin(off)
        gx = c * x - s * y + tx
        gy = s * x + c * y + ty
        return [gx, gy, wrap_angle(th + off)]

    def to_odom(self, global_pose: Pose) -> List[float]:
        """场地系位姿 -> 里程计系位姿 [x, y, theta] (go_to_global 用)."""
        gx, gy, gth = (
            float(global_pose[0]),
            float(global_pose[1]),
            float(global_pose[2]),
        )
        with self._lock:
            tx, ty, off = self._tx, self._ty, self._theta_off
        c, s = math.cos(off), math.sin(off)
        dx, dy = gx - tx, gy - ty
        # odom = R(-off) @ d
        return [c * dx + s * dy, -s * dx + c * dy, wrap_angle(gth - off)]

    # ------------------------------------------------------------------
    # 锚定 / reset 保全
    # ------------------------------------------------------------------
    def anchor(self, global_pose: Pose, odom_pose: Pose = None):
        """重锚: 声明"车现在的全局位姿是 global_pose".

        参数:
            global_pose: 车当前在场地系的位姿 [x, y, theta]
            odom_pose:   对应的里程计位姿; None=现读 odom_getter
        场景: 把车摆到场地已知点(如出发点), 调 anchor([0,0,0]) 建立场地系;
              或视觉识别到已知地标, 用地标坐标 anchor 修正漂移。
        """
        if odom_pose is None:
            odom_pose = self._get_odom()
        x, y, th = float(odom_pose[0]), float(odom_pose[1]), float(odom_pose[2])
        gx, gy, gth = (
            float(global_pose[0]),
            float(global_pose[1]),
            float(global_pose[2]),
        )
        # 解 R(off)@odom + t = global: off = gth - th; t = global - R(off)@odom
        off = wrap_angle(gth - th)
        c, s = math.cos(off), math.sin(off)
        tx = gx - (c * x - s * y)
        ty = gy - (s * x + c * y)
        with self._lock:
            self._tx, self._ty, self._theta_off = tx, ty, off

    def on_odom_reset(self, old_odom: Pose, new_odom: Pose):
        """里程计被 reset 时调用: 车没动, 全局位姿必须保持连续.

        原理: 用旧系算出 reset 瞬间的全局位姿, 再以新系重锚到同一全局位姿。
        """
        g = self.to_global(old_odom)
        self.anchor(g, new_odom)

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------
    def get_transform(self):
        """返回变换参数 (tx, ty, theta_off): global = R(off)@odom + t."""
        with self._lock:
            return (self._tx, self._ty, self._theta_off)

    def __repr__(self):
        tx, ty, off = self.get_transform()
        return (
            f"GlobalPose(tx={tx:.3f}m, ty={ty:.3f}m, "
            f"theta_off={math.degrees(off):.1f}deg)"
        )


if __name__ == "__main__":
    # 无硬件自测: mock 一个里程计, 验证锚定/reset 连续性/正逆变换
    odom = [0.0, 0.0, 0.0]
    gp = GlobalPose(odom_getter=lambda: odom)

    # 1) 车在场地 (2, 1) 朝 90° 时开机 -> 锚定场地系
    odom = [0.0, 0.0, 0.0]
    gp.anchor([2.0, 1.0, math.pi / 2])
    assert all(
        abs(a - b) < 1e-9 for a, b in zip(gp.to_global(odom), [2.0, 1.0, math.pi / 2])
    ), "锚定失败"

    # 2) 车走 1m (里程计系 +x): 全局应为朝 90° 走 -> x 不变, y+1
    odom = [1.0, 0.0, math.pi / 2]
    g = gp.to_global(odom)
    assert abs(g[0] - 2.0) < 1e-9 and abs(g[1] - 2.0) < 1e-9, f"前进步映射错: {g}"

    # 3) 里程计被清零(hooks/启动都会干): 全局必须不变
    old = odom
    odom = [0.0, 0.0, 0.0]
    gp.on_odom_reset(old, odom)
    g2 = gp.to_global(odom)
    assert (
        abs(g2[0] - 2.0) < 1e-9 and abs(g2[1] - 2.0) < 1e-9
    ), f"reset 后全局跳变: {g2}"

    # 4) 正逆变换互逆
    g3 = [3.0, -0.5, math.radians(-120.0)]
    o3 = gp.to_odom(g3)
    back = gp.to_global(o3)
    assert (
        abs(back[0] - g3[0]) < 1e-9
        and abs(back[1] - g3[1]) < 1e-9
        and abs(back[2] - g3[2]) < 1e-9
    ), f"正逆不互逆: {g3} -> {o3} -> {back}"

    # 5) 多次 reset 连续性: 每清一次零, 全局原地不动
    for _ in range(3):
        gp.on_odom_reset(odom, [0.0, 0.0, 0.0])
    g4 = gp.to_global([0.0, 0.0, 0.0])
    assert abs(g4[1] - 2.0) < 1e-9, f"多次 reset 漂移: {g4}"

    print("GlobalPose 自测全部通过:", gp)
