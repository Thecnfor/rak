// protocol.cpp —— 线路帧编解码实现
#include "protocol.h"

namespace proto {

std::vector<uint8_t> pack_mc_frame(const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 4);
    frame.push_back(HEAD0);
    frame.push_back(HEAD1);
    frame.push_back(static_cast<uint8_t>(payload.size() + 4));  // total_len = payload+4
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(TAIL);
    return frame;
}

void FrameParser::append(const uint8_t* data, size_t n) {
    buf_.insert(buf_.end(), data, data + n);
}

bool FrameParser::next(std::vector<uint8_t>& payload) {
    size_t start = 0;
    const size_t n = buf_.size();
    while (true) {
        if (n - start < 3) {
            // 不足 3 字节, 保留等待更多
            if (start > 0) buf_.erase(buf_.begin(), buf_.begin() + start);
            return false;
        }
        if (buf_[start] != HEAD0 || buf_[start + 1] != HEAD1) {
            // 帧头不对: 跳过 1 字节脏数据, 避免状态机卡死
            start += 1;
            continue;
        }
        const uint8_t total = buf_[start + 2];
        if (total < 4 || n - start < total) {
            // 长度字段非法或数据未到齐: 保留等待更多
            if (start > 0) buf_.erase(buf_.begin(), buf_.begin() + start);
            return false;
        }
        if (buf_[start + total - 1] != TAIL) {
            // 帧尾不对: 丢弃该"帧头"继续扫描
            start += 1;
            continue;
        }
        payload.assign(buf_.begin() + start + 3, buf_.begin() + start + total - 1);
        buf_.erase(buf_.begin(), buf_.begin() + start + total);
        return true;
    }
}

namespace mc602 {

// motor4: dev_id=0x01, format="bbbbb" -> "<bbbbbb", 6 字节槽位。
// get_bytes 流程(真实链路 port_id=None): data=[dev, mode=2] (2 元素),
// 剩余槽位 d_len = 6-2 = 4, 调用方给 4 个轮速 -> 全部保留上线。
// (若显式传 port_id=0 才会变成 [dev,mode,port]+3 轮速的形态 —— 真实链路不这么走)
std::vector<uint8_t> motor4_speed_payload(const int8_t negated[4]) {
    std::vector<uint8_t> p;
    p.reserve(6);
    p.push_back(0x01);  // dev_id
    p.push_back(0x02);  // mode=2 (set)
    // 无 port 字节: 4 个轮速全部上线
    for (int i = 0; i < 4; ++i) {
        p.push_back(static_cast<uint8_t>(negated[i]));
    }
    return p;
}

// encoder: dev_id=0x04, format="bbi" -> "<bbbi", 7 字节槽位。
// data=[dev, mode=1, port], 剩余槽位 d_len = 4-3 = 1, 补 1 个 0 占位 -> 7 字节。
std::vector<uint8_t> encoder_read_all() {
    std::vector<uint8_t> all;
    all.reserve(4 * 7);
    for (uint8_t port = 1; port <= 4; ++port) {
        all.push_back(0x04);  // dev_id
        all.push_back(0x01);  // mode=1 (get)
        all.push_back(port);  // port 1..4
        all.push_back(0x00);  // 4 字节 int32 占位
        all.push_back(0x00);
        all.push_back(0x00);
        all.push_back(0x00);
    }
    return all;
}

}  // namespace mc602

}  // namespace proto
