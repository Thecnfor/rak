#pragma once
// util.h —— 通用小工具: 单调时钟 / 日志 / hex 编解码
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>
#include <ctime>

// 单调时钟(秒) —— 与 Python time.monotonic() 同语义, 用于 PID dt 与超时判断
inline double mono_now() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// 日志: [HH:MM:SS.mmm] 级别 消息; 线程安全(内部加锁)
void rtd_log(const char* level, const char* fmt, ...)
    __attribute__((format(printf, 2, 3)));
#define LOGI(...) rtd_log("I", __VA_ARGS__)
#define LOGW(...) rtd_log("W", __VA_ARGS__)
#define LOGE(...) rtd_log("E", __VA_ARGS__)

// 字节数组 -> 小写 hex 串, 如 "77 68 0a" 的紧凑形式 "77680a"
std::string hex_encode(const uint8_t* d, size_t n);
inline std::string hex_encode(const std::vector<uint8_t>& v) {
    return hex_encode(v.data(), v.size());
}
// hex 串(允许空白分隔) -> 字节数组; 失败返回 false
bool hex_decode(const std::string& s, std::vector<uint8_t>& out);

// 小端读写 int32
inline int32_t le32_read(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                                (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24));
}
inline void le32_write(uint8_t* p, int32_t v) {
    uint32_t u = static_cast<uint32_t>(v);
    p[0] = static_cast<uint8_t>(u & 0xff);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xff);
    p[2] = static_cast<uint8_t>((u >> 16) & 0xff);
    p[3] = static_cast<uint8_t>((u >> 24) & 0xff);
}
