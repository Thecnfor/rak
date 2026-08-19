#pragma once
// kinematics.h —— 运动学/里程计/轮速换算 (复刻 MecanumChassis / Odometry / WheelWrap / Motors)
#include <cstdint>
#include <cmath>

// 构建自检(在 main 启动时调用, 验证换算/帧字节与 Python 复算一致)
void kinematics_selftest();

// ---------------------------------------------------------------------------
// 数值常量集中定义 + 来源公式
namespace kin {

// MecanumChassis 默认参数 (MecanumDriver.load_default_config)
//   track=0.30, wheel_base=0.28, wheel_radius=0.03
inline constexpr double HALF_TRACK = 0.15;      // 0.30/2
inline constexpr double HALF_WHEEL_BASE = 0.14; // 0.28/2
inline constexpr double WHEEL_RADIUS = 0.03;    // raduis

// roller_angle = pi/4 * 1.052 (现场标定的辊子角, 非标准 45°)
inline constexpr double ROLLER_ANGLE = M_PI / 4 * 1.052;
// tan_roller 为编译期计算
inline constexpr double TAN_ROLLER = std::tan(ROLLER_ANGLE);
// wc = half_track*tan_roller + half_wheel_base
inline constexpr double WHEEL_CONST = HALF_TRACK * TAN_ROLLER + HALF_WHEEL_BASE;

// 轮速换算常量:
//   rad2virtual = 48*(28/11)^4 / (2π) / 100 = 3.204498   (现场标定值)
//     = encoder_resolution(2015.1279) / (2π) / encoder2sp(100)
//   encoder2rad = 1/320.44975                            (现场标定值)
inline constexpr double RAD2VIRTUAL = 3.204498;
inline constexpr double ENC2RAD = 1.0 / 320.44975;

// 逆解: 车速度 [vx,vy,wz] -> 4 轮线速度 (MecanumChassis.inverse_kinematics)
//   wheel0 =  vx + vy*tan + wz*wc
//   wheel1 = -vx + vy*tan + wz*wc
//   wheel2 = -vx - vy*tan + wz*wc
//   wheel3 =  vx - vy*tan + wz*wc
inline void inverse_kinematics(const double v[3], double wheel[4]) {
    wheel[0] = v[0] + v[1] * TAN_ROLLER + v[2] * WHEEL_CONST;
    wheel[1] = -v[0] + v[1] * TAN_ROLLER + v[2] * WHEEL_CONST;
    wheel[2] = -v[0] - v[1] * TAN_ROLLER + v[2] * WHEEL_CONST;
    wheel[3] = v[0] - v[1] * TAN_ROLLER + v[2] * WHEEL_CONST;
}

// 正解: 4 轮位移 -> 车位移 (MecanumChassis.forward_kinematics)
//   dx  = (d0 - d1 - d2 + d3)/4
//   dy  = (d0 + d1 - d2 - d3)/(4*tan)
//   dth = (d0 + d1 + d2 + d3)/(4*wc)
inline void forward_kinematics(const double d[4], double out[3]) {
    out[0] = (d[0] - d[1] - d[2] + d[3]) / 4.0;
    out[1] = (d[0] + d[1] - d[2] - d[3]) / (4.0 * TAN_ROLLER);
    out[2] = (d[0] + d[1] + d[2] + d[3]) / (4.0 * WHEEL_CONST);
}

// 编码器原始增量 -> 单轮线位移 (WheelWrap.get_linear 语义, reverse=false):
//   linear_disp = -enc_delta * (1/320.44975) * 0.03
//   注意取负 —— 复刻现场标定行为(与轮速侧取负成对, 保证正转=正里程)
inline double enc_delta_to_linear(int32_t enc_delta) {
    return -static_cast<double>(enc_delta) * ENC2RAD * WHEEL_RADIUS;
}

// 4 轮线速度 -> 上线 4 个 int8 (WheelWrap.set_linear + Motors.set_angular + get_bytes):
//   1) virtual = clip(wheel_linear * (1/0.03) * 3.204498, -100, 100) 截断取 int8
//      (np.astype(int8) 是向零截断, 非四舍五入)
//   2) reverse=false: 全部取负
//   3) get_bytes: 4 个轮速全部上线 (motor4 无 port 字节, 无丢第一个怪癖)
// wire 即 motor4 帧里的 4 个轮速字节(已取负)。示例(vx=0.1):
//   wheel_linear=[0.1,-0.1,-0.1,0.1] -> virtual=[10,-10,-10,10] -> 取负[-10,10,10,-10]
//   -> wire=[-10,10,10,-10] = f6 0a 0a f6
inline void wheel_linear_to_wire(const double wheel_linear[4], int8_t wire[4]) {
    for (int i = 0; i < 4; ++i) {
        double f = wheel_linear[i] * (1.0 / WHEEL_RADIUS) * RAD2VIRTUAL;
        if (f > 100.0) f = 100.0;
        if (f < -100.0) f = -100.0;
        int8_t v = static_cast<int8_t>(static_cast<int64_t>(f));  // 截断取整
        wire[i] = static_cast<int8_t>(-v);  // 全部取负 (reverse=false 语义)
    }
}

}  // namespace kin

// ---------------------------------------------------------------------------
// 里程计 (复刻 Odometry, 世界坐标系位姿 + distance)
struct Odometry {
    double x = 0.0, y = 0.0, th = 0.0;  // position [x, y, theta] (米, 弧度)
    double distance = 0.0;              // 路程 (米)

    // Odometry.update(d_vector=[dx,dy,dth], 车辆坐标系):
    //   dx' = dx*cosθ - dy*sinθ
    //   dy' = dx*sinθ + dy*cosθ
    //   distance += hypot(dx, dy)
    void update(const double d[3]) {
        const double c = std::cos(th), s = std::sin(th);
        const double dxp = d[0] * c - d[1] * s;
        const double dyp = d[0] * s + d[1] * c;
        distance += std::hypot(d[0], d[1]);
        x += dxp;
        y += dyp;
        th += d[2];
    }

    // Odometry.reset(x=None,y=None,z=None,distance=None): 缺省=保持原值
    void reset(const double* nx, const double* ny, const double* nz,
               const double* nd) {
        if (nx) x = *nx;
        if (ny) y = *ny;
        if (nz) th = *nz;
        if (nd) distance = *nd;
    }
};
