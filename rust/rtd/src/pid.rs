// pid.rs —— PID 控制器, 语义复刻 smartcar/whalesbot/tools/tools_class.py 的 PID
// (即 simple-pid, 但本仓库默认 differential_on_measurement=True)。
//
// 关键语义(逐条对照 Python 源码):
//   - sample_time=0.01: dt<sample_time 且已有上次输出 -> 直接返回上次输出(不更新)
//     (100Hz tick 下 dt≈0.01 恰好每 tick 更新; dt 由真实单调时钟测量, 与 Python 一致)
//   - differential_on_measurement=true: derivative = -Kd*(input-last_input)/dt
//     首次调用 last_input=input -> d=0
//   - 积分: integral += Ki*error*dt, 并 clamp 到 output_limits(防饱和)
//   - 误差符号翻转时积分清零: 上次 error>0 本次<0 或反之 (注意 _last_error 初始为 0)
//   - 输出 clamp 到 output_limits; P = Kp*(setpoint-input)
//   - 设置 output_limits 时会同时 clamp 当前 integral 与 last_output (simple-pid 行为)

use crate::util::mono_now;

/// 简单 PID, 全部为安全 Rust(时间源用单调时钟)。
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub setpoint: f64,
    sample_time: f64,
    auto_mode: bool,
    proportional_on_measurement: bool,
    differential_on_measurement: bool,
    min_out: f64,
    max_out: f64,
    _proportional: f64,
    _integral: f64,
    _derivative: f64,
    _last_time: f64,
    _last_output: f64,
    has_last_output: bool,
    _last_error: f64,
    _last_input: f64,
    has_last_input: bool,
}

impl Pid {
    /// 对应 Python __init__。min_out/max_out 传 ±INFINITY 表示不限幅。
    pub fn new(
        kp: f64,
        ki: f64,
        kd: f64,
        setpoint: f64,
        min_out: f64,
        max_out: f64,
        sample_time: f64,
    ) -> Self {
        let mut p = Self {
            kp,
            ki,
            kd,
            setpoint,
            sample_time,
            auto_mode: true,
            proportional_on_measurement: false,
            differential_on_measurement: true,
            min_out,
            max_out,
            _proportional: 0.0,
            _integral: 0.0,
            _derivative: 0.0,
            _last_time: mono_now(),
            _last_output: 0.0,
            has_last_output: false,
            _last_error: 0.0,
            _last_input: 0.0,
            has_last_input: false,
        };
        // Python 末行: _integral = clamp(starting_output=0, limits) = 0
        p._integral = p.clamp(0.0);
        p
    }

    #[inline]
    fn clamp(&self, v: f64) -> f64 {
        let mut v = v;
        if self.max_out.is_finite() && v > self.max_out {
            v = self.max_out;
        }
        if self.min_out.is_finite() && v < self.min_out {
            v = self.min_out;
        }
        v
    }

    /// 对应 Python __call__(input_)。
    pub fn call(&mut self, input: f64) -> f64 {
        if !self.auto_mode {
            return if self.has_last_output { self._last_output } else { 0.0 };
        }
        let now = mono_now();
        let mut dt = now - self._last_time;
        if dt == 0.0 {
            dt = 1e-16; // Python: dt = now-last_time if 非0 else 1e-16
        }
        if self.sample_time > 0.0 && self.has_last_output && dt < self.sample_time {
            return self._last_output; // 不到采样周期: 直接返回上次输出, 不更新状态
        }
        let error = self.setpoint - input;
        let d_input = input - if self.has_last_input { self._last_input } else { input };
        // P 项: 默认 proportional_on_measurement=false -> Kp*error
        if !self.proportional_on_measurement {
            self._proportional = self.kp * error;
        } else {
            self._proportional -= self.kp * d_input;
        }
        // 积分清零: 误差符号翻转
        if self._last_error > 0.0 && error < 0.0 {
            self._integral = 0.0;
        } else if self._last_error < 0.0 && error > 0.0 {
            self._integral = 0.0;
        }
        self._integral += self.ki * error * dt;
        self._integral = self.clamp(self._integral); // 防饱和(与输出限幅一致)
        if self.differential_on_measurement {
            self._derivative = -self.kd * d_input / dt;
        } else {
            self._derivative = self.kd * (error - self._last_error) / dt; // d_error
        }
        let mut output = self._proportional + self._integral + self._derivative;
        output = self.clamp(output);
        self.has_last_output = true;
        self._last_output = output;
        self._last_input = input;
        self.has_last_input = true;
        self._last_error = error;
        self._last_time = now;
        output
    }

    /// 对应 Python 的 output_limits setter: 同时 clamp integral 与 last_output。
    pub fn set_output_limits(&mut self, min_out: f64, max_out: f64) {
        self.min_out = min_out;
        self.max_out = max_out;
        self._integral = self.clamp(self._integral);
        if self.has_last_output {
            self._last_output = self.clamp(self._last_output);
        }
    }
}
