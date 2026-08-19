// proto.rs —— MC 串口线路帧层 (复刻 smartcar/whalesbot/vehicle/base/serial_protocol.py)
//
// MC601/MC602 共用线路帧格式:
//   发送帧: 77 68 <len> <payload...> 0A   (len = len(payload)+4, 整帧长度含头尾)
//   应答帧: 同格式, 由接收侧解析
// 本模块无任何硬件/外部依赖。

pub const MC_HEADER: [u8; 2] = [0x77, 0x68];
pub const MC_TAIL: u8 = 0x0A;

/// 把内部命令字节 payload 打包为线路帧: 77 68 <len=payload+4> <payload> 0A
pub fn pack_mc_frame(payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(payload.len() + 4);
    frame.extend_from_slice(&MC_HEADER);
    frame.push((payload.len() + 4) as u8);
    frame.extend_from_slice(payload);
    frame.push(MC_TAIL);
    frame
}

/// 字节流 -> 完整帧 payload 的切帧器, 语义完全复刻 parse_mc_stream:
///   - 不足 3 字节或长度不足: 保留等待更多字节(不消费)
///   - 帧头不对: 跳过 1 字节继续扫(避免脏字节卡死状态机)
///   - 帧尾不对: 丢弃该"帧头", 跳 1 字节继续扫
///   - total<4(长度字段非法): 保留等待更多
pub struct FrameParser {
    buf: Vec<u8>,
    start: usize,
}

impl FrameParser {
    pub fn new() -> Self {
        Self {
            buf: Vec::new(),
            start: 0,
        }
    }

    /// 追加收到的字节
    pub fn append(&mut self, data: &[u8]) {
        self.buf.extend_from_slice(data);
    }

    /// 尝试切出下一帧 payload; 无完整帧返回 None。
    pub fn next(&mut self) -> Option<Vec<u8>> {
        loop {
            let n = self.buf.len();
            if n - self.start < 3 {
                return None; // 不足 3 字节, 等待更多
            }
            if self.buf[self.start] != MC_HEADER[0] || self.buf[self.start + 1] != MC_HEADER[1] {
                // 帧头不对: 跳过 1 字节脏数据
                self.start += 1;
                continue;
            }
            let total = self.buf[self.start + 2] as usize;
            if total < 4 || n - self.start < total {
                return None; // 长度非法或未到齐, 等待更多
            }
            let end = self.start + total;
            if self.buf[end - 1] != MC_TAIL {
                // 帧尾不对: 丢弃该"帧头"继续扫描
                self.start += 1;
                continue;
            }
            let payload = self.buf[self.start + 3..end - 1].to_vec();
            // 消费掉已切出的整帧
            self.buf.drain(0..end);
            self.start = 0;
            return Some(payload);
        }
    }
}

// ---------------------------------------------------------------------------
// MC602 设备命令帧构造 (复刻 mc602_devbase.DevCmdInterface.get_bytes / StructData)
//
// 帧 payload = <dev_id> <mode> [<port>] <data...>, 全部小端。
// 注意 get_bytes 的 port else 分支只减 arg_reg、不补 0:
//   - 设备设了 port_id(如 EncoderMotor_2)  -> 帧里有 port 字节
//   - 设备没设 port_id(如 Motor4_2)        -> 帧里没有 port 字节
// ---------------------------------------------------------------------------

/// motor4 轮速帧 payload: 输入 4 个 int8 虚拟速度(调用方已按 reverse=false 取负),
/// 全部上线 -> payload = 01 02 <s1> <s2> <s3> <s4> (无 port 字节)。
/// 示例(get_bytes 层, 输入为已取负后的虚拟值): [31,-32,-33,34] -> 01 02 1f e0 df 22
pub fn motor4_speed_payload(negated: &[i8; 4]) -> Vec<u8> {
    let mut p = Vec::with_capacity(6);
    p.push(0x01); // dev_id
    p.push(0x02); // mode=2 (set)
    for i in 0..4 {
        p.push(negated[i] as u8); // 无 port 字节: 4 个轮速全部上线
    }
    p
}

/// 单路编码器读 payload = 04 01 <port> 00 00 00 00 (port=1..4, 含 4 字节 int32 占位)。
pub fn encoder_read_payload(port: u8) -> Vec<u8> {
    vec![0x04, 0x01, port, 0x00, 0x00, 0x00, 0x00]
}

/// 4 帧编码器读拼成一次 write 的线路字节(4 x 帧长 11 = 44 字节;
/// 每帧 7 字节 payload + 帧头 2 + 长度 1 + 帧尾 1)。
pub fn encoder_read_all_line() -> Vec<u8> {
    let mut line = Vec::with_capacity(4 * 11);
    for port in 1..=4u8 {
        line.extend_from_slice(&pack_mc_frame(&encoder_read_payload(port)));
    }
    line
}
