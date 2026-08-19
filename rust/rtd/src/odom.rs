// odom.rs —— 里程计 (复刻 MecanumChassis 里的 Odometry, 世界坐标系位姿 + distance)
//
// Odometry.update(d_vector=[dx,dy,dth], 车辆坐标系):
//   dx' = dx*cosθ - dy*sinθ
//   dy' = dx*sinθ + dy*cosθ
//   distance += hypot(dx, dy)
//   position += [dx', dy', dth]
// Odometry.reset(x=None,y=None,z=None,distance=None): 字段缺省 = 保持原值

/// 里程计状态, 全部用 f64(与 cpp/rtd 一致; Python 侧 np.float32 的精度差异
/// 由现场标定吸收, 此处按参考实现的 double 语义处理)。
pub struct Odom {
    pub x: f64, // 世界 x (米)
    pub y: f64, // 世界 y (米)
    pub th: f64, // 航向角 (弧度)
    pub distance: f64, // 累计路程 (米)
}

impl Odom {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            th: 0.0,
            distance: 0.0,
        }
    }

    /// 车辆坐标系位移 [dx, dy, dth] -> 世界坐标系位姿更新。
    pub fn update(&mut self, d: &[f64; 3]) {
        let c = self.th.cos();
        let s = self.th.sin();
        let dxp = d[0] * c - d[1] * s;
        let dyp = d[0] * s + d[1] * c;
        self.distance += d[0].hypot(d[1]);
        self.x += dxp;
        self.y += dyp;
        self.th += d[2];
    }

    /// 重置, 缺省(不传)=保持原值。
    pub fn reset(
        &mut self,
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        distance: Option<f64>,
    ) {
        if let Some(v) = x {
            self.x = v;
        }
        if let Some(v) = y {
            self.y = v;
        }
        if let Some(v) = z {
            self.th = v;
        }
        if let Some(v) = distance {
            self.distance = v;
        }
    }
}

impl Default for Odom {
    fn default() -> Self {
        Self::new()
    }
}
