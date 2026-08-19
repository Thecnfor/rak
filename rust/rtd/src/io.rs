// io.rs —— 串口抽象: 真实 TTY(termios) / --simulate 虚拟设备 / 绝对节拍睡眠
//
// ⚠️ 本模块是 crate 内**唯一**出现 unsafe 的模块, 集中说明如下:
//   (1) SerialTty::open: 打开 /dev/ttyUSB0 + ioctl TIOCEXCL 独占 + termios
//       (B1000000 / cfmakeraw / CRTSCTS off / ICANON off / 非阻塞) —— 复刻
//       cpp/rtd/src/serial.cpp 的 SerialTty, 只操作原始 fd, 不涉及内存安全。
//   (2) SerialTty::write_all / try_read: 对已打开的 fd 做 write/read/poll,
//       缓冲区为调用方提供的切片, 长度正确, 无悬垂指针。
//   (3) sleep_until_abs: clock_gettime(CLOCK_MONOTONIC, 一次, 缓存基准) +
//       clock_nanosleep(TIMER_ABSTIME) —— 复刻控制主循环的绝对节拍语义,
//       timespec 由 std::time::Instant 换算, 计算过程安全, 仅 syscall 本身 unsafe。
// 其余所有模块(util/proto/kin/pid/odom/core/zmq/main)均为安全 Rust。

use crate::log_e;
use crate::log_i;
use crate::proto::{FrameParser, pack_mc_frame};
use std::collections::VecDeque;
use std::os::unix::io::RawFd;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// 抽象 IO: 真实串口与虚拟设备共用
// ---------------------------------------------------------------------------

pub trait Io: Send {
    /// 全量写出(处理 EINTR / 部分写)。
    fn write_all(&mut self, data: &[u8]) -> Result<(), String>;
    /// 非阻塞读取可用字节到 buf, 返回读取字节数(0 表示暂无数据)。
    /// 内部会做最多 1ms 的等待(poll 或 sleep), 避免预算窗口内忙转。
    fn try_read(&mut self, buf: &mut [u8]) -> usize;
    /// 是否虚拟设备(simulate 模式)。
    fn simulate(&self) -> bool {
        false
    }
    /// 把本轮逆解出的 4 轮线速度喂给虚拟 plant(仅 simulate 用)。
    fn set_plant_linear(&mut self, _lin: &[f64; 4]) {}
    /// 关闭底层资源。
    fn close(&mut self) {}
}

// ---------------------------------------------------------------------------
// 绝对节拍睡眠 (unsafe 集中点 3)
// ---------------------------------------------------------------------------

/// CLOCK_MONOTONIC 的基准: (对应的 Instant, 对应的 timespec)。
static CLOCK_BASE: OnceLock<(Instant, libc::timespec)> = OnceLock::new();

/// 取单调时钟基准(首次调用时用一次 clock_gettime 记录)。
fn clock_base() -> (Instant, libc::timespec) {
    *CLOCK_BASE.get_or_init(|| unsafe {
        let mut ts: libc::timespec = std::mem::zeroed();
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
        (Instant::now(), ts)
    })
}

/// 睡到绝对时刻 `next`(clock_nanosleep TIMER_ABSTIME)。
/// 语义: mono_now() 时刻与 sleep_until_abs 睡醒时刻严格对齐(同源 CLOCK_MONOTONIC)。
/// 返回时已过 next; 若 next 已过去(节拍过冲), 立即返回由调用方决定是否追帧。
pub fn sleep_until_abs(next: Instant) {
    let (base_instant, base_ts) = clock_base();
    let elapsed = next.saturating_duration_since(base_instant);
    let mut sec = base_ts.tv_sec as i64 + elapsed.as_secs() as i64;
    let mut nsec = base_ts.tv_nsec as i64 + elapsed.subsec_nanos() as i64;
    if nsec >= 1_000_000_000 {
        nsec -= 1_000_000_000;
        sec += 1;
    }
    let target = libc::timespec {
        tv_sec: sec,
        tv_nsec: nsec as libc::c_long,
    };
    unsafe {
        libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &target, std::ptr::null_mut());
    }
}

// ---------------------------------------------------------------------------
// SerialTty —— 真实串口 (unsafe 集中点 1/2)
// ---------------------------------------------------------------------------

pub struct SerialTty {
    fd: RawFd,
    path: String,
}

impl SerialTty {
    /// 打开串口: O_RDWR | O_NOCTTY + TIOCEXCL 独占 + 1,000,000 波特 8N1 raw。
    pub fn open(path: &str) -> Result<Self, String> {
        let cpath = std::ffi::CString::new(path)
            .map_err(|_| format!("路径含 NUL: {}", path))?;
        // unsafe 1a: open (flags 为普通整型常量, 不涉及内存安全)
        let fd = unsafe { libc::open(cpath.as_ptr(), libc::O_RDWR | libc::O_NOCTTY) };
        if fd < 0 {
            return Err(format!("打开串口 {} 失败: {}", path, std::io::Error::last_os_error()));
        }

        // unsafe 1b: TIOCEXCL 独占, 防止其他进程(如 Python)同时打开
        unsafe {
            let excl: libc::c_int = 1;
            libc::ioctl(fd, libc::TIOCEXCL, &excl);
        }

        // unsafe 1c: termios 配置(逐项复刻 cpp/rtd/src/serial.cpp SerialTty)
        unsafe {
            let mut tio: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut tio) != 0 {
                libc::close(fd);
                return Err(format!("tcgetattr 失败: {}", std::io::Error::last_os_error()));
            }
            // 1,000,000 波特
            libc::cfsetispeed(&mut tio, libc::B1000000);
            libc::cfsetospeed(&mut tio, libc::B1000000);
            // raw 模式: ICANON/ECHO/ISIG/IXON 全部关闭
            libc::cfmakeraw(&mut tio);
            // 硬件流控 off, 忽略调制解调器控制, 使能接收
            tio.c_cflag &= !(libc::CRTSCTS as libc::tcflag_t);
            tio.c_cflag |= (libc::CLOCAL | libc::CREAD) as libc::tcflag_t;
            // 非阻塞读: poll + read 循环(poll 由 try_read 内做 1ms 等待)
            tio.c_cc[libc::VMIN] = 0;
            tio.c_cc[libc::VTIME] = 0;
            if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
                libc::close(fd);
                return Err(format!("tcsetattr 失败: {}", std::io::Error::last_os_error()));
            }
            libc::tcflush(fd, libc::TCIOFLUSH);
        }

        // unsafe 1d: fcntl 置 O_NONBLOCK, 使 read 无数据时立即返回 0(EAGAIN)
        unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFL);
            if flags >= 0 {
                libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
            }
        }

        log_i!("串口 {} 已打开 (1000000 8N1 raw, TIOCEXCL 独占)", path);
        Ok(Self {
            fd,
            path: path.to_string(),
        })
    }
}

impl Io for SerialTty {
    fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        let mut off = 0usize;
        while off < data.len() {
            // unsafe 2a: write 原始 fd; 切片指针仅在调用期间有效, 长度正确
            let n = unsafe {
                libc::write(
                    self.fd,
                    data.as_ptr().add(off) as *const libc::c_void,
                    data.len() - off,
                )
            };
            if n < 0 {
                let e = std::io::Error::last_os_error();
                if e.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(format!("串口写失败: {}", e));
            }
            off += n as usize;
        }
        Ok(())
    }

    fn try_read(&mut self, buf: &mut [u8]) -> usize {
        // unsafe 2b: poll 最多 1ms 等可读, 避免 3ms 预算窗口内忙转
        let mut pfd = libc::pollfd {
            fd: self.fd,
            events: libc::POLLIN,
            revents: 0,
        };
        let pr = unsafe { libc::poll(&mut pfd, 1, 1) };
        if pr <= 0 || (pfd.revents & libc::POLLIN) == 0 {
            return 0;
        }
        // unsafe 2c: read 非阻塞, 缓冲区为调用方切片
        let n = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n > 0 {
            n as usize
        } else {
            0
        }
    }

    fn close(&mut self) {
        if self.fd >= 0 {
            // unsafe: 释放 TIOCEXCL 独占后 close
            unsafe {
                let excl: libc::c_int = 0;
                libc::ioctl(self.fd, libc::TIOCEXCL, &excl);
                libc::close(self.fd);
            }
            self.fd = -1;
            log_i!("串口 {} 已关闭", self.path);
        }
    }
}

// ---------------------------------------------------------------------------
// SimDevice —— --simulate 虚拟设备: 不碰真实串口
//   - 写进来的是线路帧(编码器读/轮速/透传), 虚拟设备解析后向内存队列注入应答帧
//   - 编码器回放: 根据最近一次"下发的轮速"(plant 线速度)积分原始编码值,
//     使里程计在模拟下跟随指令, 可用于联调 vel/goto/reset 闭环
// 本实现用内存队列代替 C++ 的 pipe, 全部安全 Rust。
// ---------------------------------------------------------------------------

pub struct SimDevice {
    rx_queue: VecDeque<u8>, // 控制线程读取的应答字节
    tick_hz: f64,
    plant_linear: [f64; 4],
    enc_raw: [i32; 4],
}

impl SimDevice {
    pub fn new(tick_hz: f64) -> Self {
        log_i!("simulate 模式: 未打开任何真实串口, 使用虚拟设备 (tick_hz={:.0})", tick_hz);
        Self {
            rx_queue: VecDeque::new(),
            tick_hz,
            plant_linear: [0.0; 4],
            enc_raw: [0; 4],
        }
    }

    /// 解析一次写入的线路帧并生成应答注入队列。
    fn on_write(&mut self, data: &[u8]) {
        let mut parser = FrameParser::new();
        parser.append(data);
        while let Some(payload) = parser.next() {
            if payload.len() < 3 {
                continue;
            }
            let (dev, mode, p2) = (payload[0], payload[1], payload[2]);
            if dev == 0x04 && mode == 0x01 && (1..=4).contains(&p2) {
                // 编码器读: 推进虚拟原始编码(每 tick 一次)
                let idx = (p2 - 1) as usize;
                // 与里程计换算互为逆过程: raw_delta = -v_linear*dt*(320.44975/0.03)
                let delta = -self.plant_linear[idx] / self.tick_hz * (320.44975 / 0.03);
                self.enc_raw[idx] = self.enc_raw[idx].wrapping_add(delta as i32);
                // 应答 payload = 04 01 <port> <int32 LE>
                let mut rep = vec![0x04, 0x01, p2, 0, 0, 0, 0];
                crate::util::le32_write(&mut rep[3..], self.enc_raw[idx]);
                let frame = pack_mc_frame(&rep);
                self.rx_queue.extend(frame);
            } else if dev == 0x01 && mode == 0x02 && payload.len() == 6 {
                // motor4 轮速帧: 反推线速度(用于 simulate 日志核对)
                let mut lin = [0.0f64; 4];
                for i in 0..4 {
                    let s = payload[2 + i] as i8;
                    lin[i] = -(s as f64) / ((1.0 / crate::kin::WHEEL_RADIUS) * crate::kin::RAD2VIRTUAL);
                }
                log_i!(
                    "[sim] motor4 帧 -> 线速度 [{:.3} {:.3} {:.3} {:.3}] m/s",
                    lin[0],
                    lin[1],
                    lin[2],
                    lin[3]
                );
            } else {
                // 透传帧: 原样回显, 便于联调 frame / frame_async / sub
                self.rx_queue.extend(pack_mc_frame(&payload));
            }
        }
    }
}

impl Io for SimDevice {
    fn write_all(&mut self, data: &[u8]) -> Result<(), String> {
        self.on_write(data);
        Ok(())
    }

    fn try_read(&mut self, buf: &mut [u8]) -> usize {
        let mut n = 0usize;
        while n < buf.len() {
            match self.rx_queue.pop_front() {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        if n == 0 {
            // 无数据时等 1ms 节流, 与 SerialTty 的 poll(1ms) 行为一致
            std::thread::sleep(Duration::from_millis(1));
        }
        n
    }

    fn simulate(&self) -> bool {
        true
    }

    fn set_plant_linear(&mut self, lin: &[f64; 4]) {
        self.plant_linear = *lin;
    }
}

// ---------------------------------------------------------------------------
// 工厂
// ---------------------------------------------------------------------------

/// simulate 为真返回 SimDevice, 否则打开真实串口(打开失败返回 None)。
pub fn create_io(simulate: bool, port: &str, tick_hz: f64) -> Option<Box<dyn Io>> {
    if simulate {
        Some(Box::new(SimDevice::new(tick_hz)))
    } else {
        match SerialTty::open(port) {
            Ok(s) => Some(Box::new(s)),
            Err(e) => {
                log_e!("{}", e);
                None
            }
        }
    }
}
