// kin.rs —— 运动学 / 里程计换算 / 轮速上线 (复刻 MecanumChassis / Odometry / WheelWrap / Motors)
//
// 数值常量集中定义, 每条注明来源公式。MecanumChassis 默认参数来自
// MecanumDriver.load_default_config: track=0.30, wheel_base=0.28, wheel_radius=0.03。

use std::f64::consts::PI;

// MecanumChassis 默认尺寸
pub const HALF_TRACK: f64 = 0.15; // 0.30/2 轮距半宽
pub const HALF_WHEEL_BASE: f64 = 0.14; // 0.28/2 轴距半长
pub const WHEEL_RADIUS: f64 = 0.03; // 轮半径 (config "raduis")

// roller_angle = pi/4 * 1.052 (现场标定的辊子角, 非标准 45°)
pub const ROLLER_ANGLE: f64 = PI / 4.0 * 1.052;
// tan_roller = tan(ROLLER_ANGLE) = 1.0852087757245255
// wc = half_track*tan_roller + half_wheel_base = 0.30278131635867883
// 二者由上面的源常量在首次调用时计算一次并缓存, 与 Python math.tan 结果一致
// (f64::tan 非 const fn, 故不能写成 const; OnceLock 只算一次, 热路径无影响)。
static TAN_ROLLER: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
static WHEEL_CONST: std::sync::OnceLock<f64> = std::sync::OnceLock::new();

#[inline]
pub fn tan_roller() -> f64 {
    *TAN_ROLLER.get_or_init(|| ROLLER_ANGLE.tan())
}

#[inline]
pub fn wheel_const() -> f64 {
    *WHEEL_CONST.get_or_init(|| HALF_TRACK * tan_roller() + HALF_WHEEL_BASE)
}

// 轮速换算常量(现场标定):
//   rad2virtual = 48*(28/11)^4 / (2π) / 100 = 3.204498
//     = encoder_resolution(2015.12792842019) / (2π) / encoder2sp(100)
//   encoder2rad = 1/320.44975  (现场标定)
pub const RAD2VIRTUAL: f64 = 3.204498;
pub const ENC2RAD: f64 = 1.0 / 320.44975; // = 0.0031206140744375679

/// 逆解: 车速度 [vx,vy,wz] -> 4 轮线速度 (MecanumChassis.inverse_kinematics)
///   wheel0 =  vx + vy*tan + wz*wc
///   wheel1 = -vx + vy*tan + wz*wc
///   wheel2 = -vx - vy*tan + wz*wc
///   wheel3 =  vx - vy*tan + wz*wc
pub fn inverse_kinematics(v: &[f64; 3]) -> [f64; 4] {
    [
        v[0] + v[1] * tan_roller() + v[2] * wheel_const(),
        -v[0] + v[1] * tan_roller() + v[2] * wheel_const(),
        -v[0] - v[1] * tan_roller() + v[2] * wheel_const(),
        v[0] - v[1] * tan_roller() + v[2] * wheel_const(),
    ]
}

/// 正解: 4 轮位移 -> 车位移 (MecanumChassis.forward_kinematics)
///   dx  = (d0 - d1 - d2 + d3)/4
///   dy  = (d0 + d1 - d2 - d3)/(4*tan)
///   dth = (d0 + d1 + d2 + d3)/(4*wc)
pub fn forward_kinematics(d: &[f64; 4]) -> [f64; 3] {
    [
        (d[0] - d[1] - d[2] + d[3]) / 4.0,
        (d[0] + d[1] - d[2] - d[3]) / (4.0 * tan_roller()),
        (d[0] + d[1] + d[2] + d[3]) / (4.0 * wheel_const()),
    ]
}

/// 编码器原始增量 -> 单轮线位移 (WheelWrap.get_linear 语义, reverse=false):
///   linear_disp = -enc_delta * (1/320.44975) * 0.03
/// 注意取负 —— 复刻现场标定行为(与轮速侧取负成对, 保证"正转=正里程")。
pub fn enc_delta_to_linear(enc_delta: i32) -> f64 {
    -(enc_delta as f64) * ENC2RAD * WHEEL_RADIUS
}

/// 4 轮线速度 -> 上线 4 个 int8 (WheelWrap.set_linear + Motors.set_speed + get_bytes):
///   1) virtual = clip(wheel_linear * (1/0.03) * 3.204498, -100, 100) 截断取 int8
///      (np.astype(int8) 是向零截断, 非四舍五入)
///   2) reverse=false: 全部取负(复刻现场标定)
///   3) 4 个轮速全部上线(motor4 无 port 字节, 无"丢第一个"怪癖)
/// 示例(vx=0.1): wheel_linear=[0.1,-0.1,-0.1,0.1] -> virtual=[10,-10,-10,10]
///   -> 取负 [-10,10,10,-10] = f6 0a 0a f6
pub fn wheel_linear_to_wire(wheel_linear: &[f64; 4]) -> [i8; 4] {
    let mut wire = [0i8; 4];
    for i in 0..4 {
        let mut f = wheel_linear[i] * (1.0 / WHEEL_RADIUS) * RAD2VIRTUAL;
        if f > 100.0 {
            f = 100.0;
        }
        if f < -100.0 {
            f = -100.0;
        }
        let v = f as i64 as i8; // 截断取整(向零), 同 np.astype(int8)
        wire[i] = -v; // 全部取负 (reverse=false 语义)
    }
    wire
}
