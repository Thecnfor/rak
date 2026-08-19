#pragma once
// pid.h —— PID 控制器, 语义复刻 smartcar/whalesbot/tools/tools_class.py 的 PID
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
#include "util.h"
#include <cmath>

struct PID {
    double Kp, Ki, Kd;
    double setpoint;
    double sample_time;
    bool auto_mode;
    bool proportional_on_measurement;
    bool differential_on_measurement;

    double min_out, max_out;  // output_limits

    double _proportional = 0;
    double _integral = 0;
    double _derivative = 0;
    double _last_time;          // 上次更新的单调时钟
    double _last_output = 0;    // 上次输出
    bool has_last_output = false;
    double _last_error = 0;     // 初始 0, 与 Python 一致(且 reset 不清除它)
    double _last_input = 0;
    bool has_last_input = false;

    // Python __init__: 构造时 reset() 会置 _last_time = 当前单调时钟
    PID(double Kp_, double Ki_, double Kd_, double setpoint_,
        double min_out_ = -INFINITY, double max_out_ = INFINITY,
        double sample_time_ = 0.01)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_), setpoint(setpoint_),
          sample_time(sample_time_), auto_mode(true),
          proportional_on_measurement(false), differential_on_measurement(true),
          min_out(min_out_), max_out(max_out_) {
        _last_time = mono_now();
        // 构造后 _integral = clamp(starting_output=0, limits) = 0 (Python 末行)
        _integral = clamp(_integral);
        if (has_last_output) _last_output = clamp(_last_output);
    }

    double clamp(double v) const {
        if (std::isfinite(max_out) && v > max_out) v = max_out;
        if (std::isfinite(min_out) && v < min_out) v = min_out;
        return v;
    }

    // 对应 Python __call__(input_)
    double call(double input) {
        if (!auto_mode) return has_last_output ? _last_output : 0.0;
        const double now = mono_now();
        double dt = now - _last_time;
        if (dt == 0.0) dt = 1e-16;  // Python: dt = now-last_time if 非0 else 1e-16
        if (sample_time > 0 && has_last_output && dt < sample_time) {
            return _last_output;  // 不到采样周期: 直接返回上次输出, 不更新状态
        }
        const double error = setpoint - input;
        const double d_input = input - (has_last_input ? _last_input : input);
        // P 项: 默认 proportional_on_measurement=false -> Kp*error
        if (!proportional_on_measurement) {
            _proportional = Kp * error;
        } else {
            _proportional -= Kp * d_input;
        }
        // 积分清零: 误差符号翻转
        if (_last_error > 0 && error < 0) {
            _integral = 0;
        } else if (_last_error < 0 && error > 0) {
            _integral = 0;
        }
        _integral += Ki * error * dt;
        _integral = clamp(_integral);  // 防饱和(与输出限幅一致)
        if (differential_on_measurement) {
            _derivative = -Kd * d_input / dt;
        } else {
            _derivative = Kd * (error - _last_error) / dt;  // d_error
        }
        double output = _proportional + _integral + _derivative;
        output = clamp(output);
        has_last_output = true;
        _last_output = output;
        _last_input = input;
        has_last_input = true;
        _last_error = error;
        _last_time = now;
        return output;
    }

    // Python 的 output_limits setter: 同时 clamp integral 与 last_output
    void set_output_limits(double mn, double mx) {
        min_out = mn;
        max_out = mx;
        _integral = clamp(_integral);
        if (has_last_output) _last_output = clamp(_last_output);
    }
};
