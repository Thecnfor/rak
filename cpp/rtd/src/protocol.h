#pragma once
// protocol.h —— MC601/MC602 串口线路帧层 (复刻 smartcar/whalesbot/vehicle/base/serial_protocol.py)
//
// 线路帧格式:
//   发送帧: 77 68 <len> <payload...> 0A   (len = len(payload)+4, 整帧长度含头尾)
//   应答帧: 同格式, 由接收侧解析
#include <cstddef>
#include <cstdint>
#include <vector>

namespace proto {

// 帧头/帧尾常量 (与 serial_protocol.py MC_HEADER / MC_TAIL 一致)
inline constexpr uint8_t HEAD0 = 0x77;
inline constexpr uint8_t HEAD1 = 0x68;
inline constexpr uint8_t TAIL = 0x0A;

// pack_mc_frame: 把 payload 打包为线路帧; payload 长度最大 251 (len 为单字节)
std::vector<uint8_t> pack_mc_frame(const std::vector<uint8_t>& payload);

// FrameParser: 字节流 -> 整帧 payload 切帧器
// 语义完全复刻 parse_mc_stream:
//   - 不足 3 字节或长度不足: 等待更多字节(不消费)
//   - 帧头不对: 跳过 1 字节继续扫
//   - 帧尾不对: 丢弃该帧头, 跳 1 字节继续
class FrameParser {
public:
    void append(const uint8_t* data, size_t n);
    void append(const std::vector<uint8_t>& d) { append(d.data(), d.size()); }
    // 取出一帧完整 payload; 无完整帧返回 false
    bool next(std::vector<uint8_t>& payload);

private:
    std::vector<uint8_t> buf_;
};

// ---------------------------------------------------------------------------
// MC602 设备命令帧构造 (复刻 mc602_devbase.DevCmdInterface.get_bytes / StructData)
//
// 帧 payload = <dev_id> <mode> [<port>] <data...>, 全部小端。
// 注意 get_bytes 的 port else 分支只减 arg_reg、不补 0:
//   - 设备设了 port_id(如 EncoderMotor_2)  -> 帧里有 port 字节
//   - 设备没设 port_id(如 Motor4_2)        -> 帧里没有 port 字节
// motor4 format="bbbbb" -> 槽位 6 字节 = dev+mode+4 个轮速, 4 个轮速全部上线,
// 没有"丢第一个"的怪癖(那是早期把 port_id=0 显式传入时才会出现的形态,
// 真实链路 Motors_2.set_speed 走 port_id=None, 已用真实 DevCmdInterface 复算验证)。
namespace mc602 {

// motor4 轮速帧 payload: 输入 4 个 int8 虚拟速度(调用方已按 reverse=false 取负),
// 全部上线 -> payload = 01 02 <s1> <s2> <s3> <s4> (无 port 字节)。
// 示例(get_bytes 层, 输入为已取负后的虚拟值): [31,-32,-33,34] -> 01 02 1f e0 df 22
std::vector<uint8_t> motor4_speed_payload(const int8_t negated[4]);

// 编码器读: 4 帧拼一次写, 每帧 payload = 04 01 <port> 00 00 00 00 (port=1..4)。
// 该帧含 4 字节占位(对应应答里的 int32), 是真实硬件读取格式。
// 返回 4 帧拼好的一次 write 内容(不含线路帧头尾由调用方 pack)。
std::vector<uint8_t> encoder_read_all();

}  // namespace mc602

}  // namespace proto
