# -*- coding: utf-8 -*-
"""巡线/车道保持(LaneMixin): 前置摄像头车道闭环控制(从 motion.py 拆分而来)。"""
import contextlib
import time

from .. import cfg  # 巡线参数统一入口(tasks/tools/cfg.py)

# 方法默认参数用到的停止标志默认值(与 MyCar.STOP_PARAM 类属性保持一致)
STOP_PARAM = True


class LaneMixin:

    # correction CNN 叠加到角速度的参数(实车标定, 统一走 tasks/tools/cfg.py)
    _corr_threshold = cfg.CORR_THRESHOLD  # |steer| 小于此值不启用(防抖)
    _corr_weight = cfg.CORR_WEIGHT  # steer 1.0 对应的角速度贡献
    # 巡线前进速度默认: lane_base(speed) 未覆盖时兜底 (speed 参数优先, 见 lane_base)
    _lane_v_min = cfg.V_FORWARD  # 兜底前进速度(m/s); 恒速, 不随误差降速
    # 转向死区(da 进角 PID 前): 实例属性, 供路段特调覆盖(cfg.PID_DEADZONE)
    _lane_deadzone = cfg.PID_DEADZONE
    # 速度分级误差上限(速度分级可选; 默认 0.3)
    _lane_err_max = 0.3

    # ------------------------------------------------------------------
    # 路段参数应用/还原: 每段路可独立调巡线参数, 跑完还原不污染全局
    # ------------------------------------------------------------------
    # lane_apply_params 可接受的字段 -> 应用目标:
    #   kp/ki/kd/limits          -> self.lane_pid_angle (转向 PID)
    #   deadzone                 -> self._lane_deadzone (da 进 PID 前的死区)
    #   corr_threshold/corr_weight -> _corr_threshold/_corr_weight (居中通道)
    #   ema/lane_timeout         -> _lane_ema/_lane_timeout (前视滤波)
    #   v_forward                -> _lane_v_min (前进速度, 恒速)
    #   err_max                  -> _lane_err_max (速度分级误差上限)
    _LANE_PARAM_TARGETS = {
        "kp": ("lane_pid_angle", "Kp"),
        "ki": ("lane_pid_angle", "Ki"),
        "kd": ("lane_pid_angle", "Kd"),
        "limits": ("lane_pid_angle", "output_limits"),
        "deadzone": (None, "_lane_deadzone"),
        "corr_threshold": (None, "_corr_threshold"),
        "corr_weight": (None, "_corr_weight"),
        "ema": (None, "_lane_ema"),
        "lane_timeout": (None, "_lane_timeout"),
        "v_forward": (None, "_lane_v_min"),
        "err_max": (None, "_lane_err_max"),
    }

    def lane_apply_params(self, params: dict) -> None:
        """把路段特调参数应用到本次巡线; 调用前先快照, 供 lane_restore_params 还原.

        params: dict, 键见 _LANE_PARAM_TARGETS, 缺省不动对应项(用全局默认)。
        仅覆盖 lane_base/lane_pid_angle 会用到的项, 不改 cfg.py 常量。
        """
        if not params:
            return
        # 快照当前生效值(只存本次要覆盖的项)
        self._lane_param_backup = {}
        for key, (obj_attr, attr) in self._LANE_PARAM_TARGETS.items():
            if key not in params:
                continue
            if obj_attr is not None:
                obj = getattr(self, obj_attr)
                self._lane_param_backup[key] = (obj_attr, attr, getattr(obj, attr))
                setattr(obj, attr, params[key])
            else:
                self._lane_param_backup[key] = (None, attr, getattr(self, attr))
                setattr(self, attr, params[key])

    def lane_restore_params(self) -> None:
        """还原 lane_apply_params 快照的参数; 无快照则不做任何事."""
        backup = getattr(self, "_lane_param_backup", None)
        if not backup:
            return
        for key, (obj_attr, attr, value) in backup.items():
            if obj_attr is not None:
                setattr(getattr(self, obj_attr), attr, value)
            else:
                setattr(self, attr, value)
        self._lane_param_backup = None

    @contextlib.contextmanager
    def lane_config(self, params: dict):
        """路段巡线参数上下文管理器: 进入应用, 退出还原(异常也还原).

        用法:
            with car.lane_config(seg):
                car.lane_base(...)
        """
        self.lane_apply_params(params)
        try:
            yield
        finally:
            self.lane_restore_params()

    # ------------------------------------------------------------------
    # 巡线主循环
    # ------------------------------------------------------------------
    def lane_base(self, speed, end_fuction, stop=STOP_PARAM):
        """
        车道保持基础方法(双模型版)

        使用前置摄像头进行车道检测和保持(源头合成新一对 steer/da):
            - 转弯: lane 模型的 d_a 进 cfg_pid_angle -> 角速度
            - 居中: correction CNN 的 steer 叠加到角速度(打方向回正, 同时
              修正"不居中"与"不平行"); y_speed 横向通道不再使用

        标定:
            _corr_threshold: |steer| 低于此值不叠加, 防抖(默认 0.05)
            _corr_weight:    steer 1.0 对应的角速度贡献(默认 0.5;
                             居中漂移 -> 调高, 跟线不稳 -> 调低, 抖动 -> 调大阈值)
        """
        v_min = getattr(self, "_lane_v_min", cfg.V_FORWARD)
        v_max = max(speed, v_min)
        corr_threshold = getattr(self, "_corr_threshold", cfg.CORR_THRESHOLD)
        corr_weight = getattr(self, "_corr_weight", cfg.CORR_WEIGHT)
        deadzone = getattr(self, "_lane_deadzone", cfg.PID_DEADZONE)

        while True:
            if self._stop_flag:
                return

            # 源头合成后的新一对: steer 居中 + da 转弯
            steer, da = self.get_lane_results()
            # 转弯通道: da 先过死区(削直线噪声), 再进角 PID -> 角速度
            # 削零式: |da|<=死区归零, 超过后按 (|da|-死区) 连续响应(转弯不突变)
            if abs(da) <= deadzone:
                da_dz = 0.0
            else:
                da_dz = (abs(da) - deadzone) * (1.0 if da > 0 else -1.0)
            angle_speed = self.lane_pid_angle(-da_dz)
            # 居中通道: correction steer 叠加到角速度(打方向回正)
            # 注意: steer>0 按打标约定=右转回正; 实车发现方向反了, 用减号
            if abs(steer) > corr_threshold:
                angle_speed -= steer * corr_weight
            # 恒速前进: 速度取 v_min(_lane_v_min, 可被 lane.v_forward 覆盖),
            # 上限 v_max 兜底(speed 或 v_min 取大), 不随误差降速
            run_speed = v_min + (v_max - v_min) * 1.0
            self.set_velocity(run_speed, 0.0, angle_speed)
            if end_fuction():
                break
        if stop:
            self.stop()

    def lane_time(self, speed, time_dur, stop=STOP_PARAM):
        """
        车道保持定时方法

        使用前置摄像头进行车道保持，持续指定的时间。

        参数:
            speed: 行驶速度
            time_dur: 持续时间（秒）
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        time_end = time.time() + time_dur

        def end_fuction():
            return time.time() > time_end

        self.lane_base(speed, end_fuction, stop=stop)

    def lane_dis(self, speed, dis_end, stop=STOP_PARAM):
        """
        车道保持定距方法

        使用前置摄像头进行车道保持，直到行驶距离超过指定值。

        参数:
            speed: 行驶速度
            dis_end: 目标距离
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """

        # lambda重新endfunction
        def end_fuction():
            return self.get_distance() > dis_end

        self.lane_base(speed, end_fuction, stop=stop)

    def lane_dis_offset(self, speed, dis_hold, stop=STOP_PARAM):
        """
        车道保持距离偏移方法

        使用前置摄像头进行车道保持，行驶指定的距离偏移量。

        参数:
            speed: 行驶速度
            dis_hold: 距离偏移量
            stop: 是否在结束后停止车辆，默认为STOP_PARAM
        """
        dis_start = self.get_distance()
        dis_stop = dis_start + dis_hold
        self.lane_dis(speed, dis_stop, stop=stop)
