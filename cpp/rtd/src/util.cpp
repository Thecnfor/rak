// util.cpp —— 日志与 hex 工具实现
#include "util.h"
#include <mutex>
#include <cstring>

namespace {
std::mutex g_log_mutex;
}  // namespace

void rtd_log(const char* level, const char* fmt, ...) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tmv;
    localtime_r(&ts.tv_sec, &tmv);
    char head[64];
    snprintf(head, sizeof(head), "[%02d:%02d:%02d.%03d] %s ",
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec,
             static_cast<int>(ts.tv_nsec / 1000000), level);
    va_list ap;
    va_start(ap, fmt);
    char msg[1024];
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    std::lock_guard<std::mutex> lk(g_log_mutex);
    fputs(head, stderr);
    fputs(msg, stderr);
    fputc('\n', stderr);
    fflush(stderr);
}

std::string hex_encode(const uint8_t* d, size_t n) {
    static const char* hex = "0123456789abcdef";
    std::string out;
    out.reserve(n * 2);
    for (size_t i = 0; i < n; ++i) {
        out.push_back(hex[d[i] >> 4]);
        out.push_back(hex[d[i] & 0x0f]);
    }
    return out;
}

bool hex_decode(const std::string& s, std::vector<uint8_t>& out) {
    out.clear();
    auto hexv = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    int hi = -1;
    for (char c : s) {
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') continue;
        int v = hexv(c);
        if (v < 0) return false;
        if (hi < 0) {
            hi = v;
        } else {
            out.push_back(static_cast<uint8_t>((hi << 4) | v));
            hi = -1;
        }
    }
    if (hi >= 0) return false;  // 奇数个 hex 字符
    return true;
}
