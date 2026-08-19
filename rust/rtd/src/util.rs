// util.rs —— 通用小工具: 单调时钟 / hex 编解码 / 小端读写 / 日志
//
// 本模块全部是安全 Rust。唯一会用到 unsafe 的底层调用(termios 串口配置、
// clock_nanosleep 绝对节拍)集中在 src/io.rs, 那里有集中说明。
//
// 单调时钟说明: 所有内部时间比较(超时/PID dt/节拍误差)都只要求"单调且一致",
// 用 std::time::Instant 即可(等价于 Python time.monotonic)。绝对节拍
// sleep_until_abs 由 src/io.rs 把同一基准 Instant 换算成 CLOCK_MONOTONIC 的
// timespec, 两者同源(Instant 在 Linux 上就由 CLOCK_MONOTONIC 驱动), 因此
// "按 mono_now() 算出的绝对时刻" 与 "clock_nanosleep 睡醒时刻" 严格对齐。

use std::sync::OnceLock;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

/// 单调时钟零点(程序首次调用 mono_now 时建立)。
static CLOCK_EPOCH: OnceLock<Instant> = OnceLock::new();

/// 单调时钟(秒) —— 与 Python time.monotonic() 同语义。
pub fn mono_now() -> f64 {
    let epoch = *CLOCK_EPOCH.get_or_init(Instant::now);
    epoch.elapsed().as_secs_f64()
}

// ---------------------------------------------------------------------------
// hex 编解码
// ---------------------------------------------------------------------------

/// 字节数组 -> 紧凑小写 hex 串, 如 "77680a"(与 ZMQ 协议约定一致)。
pub fn hex_encode(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 2);
    for b in data {
        s.push(char::from_digit((*b >> 4) as u32, 16).unwrap());
        s.push(char::from_digit((*b & 0x0f) as u32, 16).unwrap());
    }
    s
}

/// hex 串(允许空白分隔) -> 字节数组; 失败返回 None。
pub fn hex_decode(s: &str) -> Option<Vec<u8>> {
    let cleaned: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if cleaned.len() % 2 != 0 {
        return None;
    }
    let bytes = cleaned.as_bytes();
    let mut out = Vec::with_capacity(cleaned.len() / 2);
    for i in (0..bytes.len()).step_by(2) {
        let hi = hex_val(bytes[i])?;
        let lo = hex_val(bytes[i + 1])?;
        out.push((hi << 4) | lo);
    }
    Some(out)
}

fn hex_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// 小端读写 int32 (编码器应答里的 int32 LE)
// ---------------------------------------------------------------------------

#[inline]
pub fn le32_read(p: &[u8]) -> i32 {
    i32::from_le_bytes([p[0], p[1], p[2], p[3]])
}

#[inline]
pub fn le32_write(p: &mut [u8], v: i32) {
    p[..4].copy_from_slice(&v.to_le_bytes());
}

// ---------------------------------------------------------------------------
// 日志: [HH:MM:SS.mmm] 级别 消息, 线程安全(加锁防交错), 走 stderr(供 systemd 采集)
// ---------------------------------------------------------------------------

use std::sync::Mutex;

static LOG_LOCK: Mutex<()> = Mutex::new(());

fn wall_time() -> String {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = now.as_secs();
    let millis = now.subsec_millis();
    let tod = secs % 86400; // 仅显示当日时刻(UTC), 日志无需日期
    let h = tod / 3600;
    let m = (tod % 3600) / 60;
    let s = tod % 60;
    format!("{:02}:{:02}:{:02}.{:03}", h, m, s, millis)
}

pub fn rtd_log(level: &str, msg: &str) {
    let _guard = LOG_LOCK.lock().unwrap();
    eprintln!("[{}] {} {}", wall_time(), level, msg);
}

#[macro_export]
macro_rules! log_i {
    ($($arg:tt)*) => { $crate::util::rtd_log("I", &format!($($arg)*)) };
}
#[macro_export]
macro_rules! log_w {
    ($($arg:tt)*) => { $crate::util::rtd_log("W", &format!($($arg)*)) };
}
#[macro_export]
macro_rules! log_e {
    ($($arg:tt)*) => { $crate::util::rtd_log("E", &format!($($arg)*)) };
}
