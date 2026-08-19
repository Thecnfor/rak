// serial.cpp —— 串口实现
#include "serial.h"
#include "util.h"
#include "protocol.h"
#include "kinematics.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <cstring>
#include <cerrno>

// ---------------------------------------------------------------------------
// SerialTty
// ---------------------------------------------------------------------------
SerialTty::SerialTty(const std::string& path) : path_(path) {
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        LOGE("打开串口 %s 失败: %s", path.c_str(), strerror(errno));
        return;
    }
    // 独占: 防止其他进程(如 Python)同时打开
    int excl = 1;
    ioctl(fd_, TIOCEXCL, &excl);

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd_, &tio) != 0) {
        LOGE("tcgetattr 失败: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return;
    }
    // 1,000,000 波特
    cfsetispeed(&tio, B1000000);
    cfsetospeed(&tio, B1000000);
    // raw 模式: ICANON/ECHO/ISIG/IXON 全部关闭
    cfmakeraw(&tio);
    tio.c_cflag &= ~(static_cast<tcflag_t>(CRTSCTS));  // 硬件流控 off
    tio.c_cflag |= (CLOCAL | CREAD);                    // 忽略调制解调器控制, 使能接收
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;  // 非阻塞读(poll + read 循环)
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        LOGE("tcsetattr 失败: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return;
    }
    tcflush(fd_, TCIOFLUSH);
    LOGI("串口 %s 已打开 (1000000 8N1 raw, TIOCEXCL 独占)", path.c_str());
}

void SerialTty::write(const uint8_t* d, size_t n) {
    if (fd_ < 0) return;
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::write(fd_, d + off, n - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            LOGW("串口写失败: %s", strerror(errno));
            return;
        }
        off += static_cast<size_t>(w);
    }
}

void SerialTty::close() {
    if (fd_ >= 0) {
        int excl = 0;
        ioctl(fd_, TIOCEXCL, &excl);  // 释放独占
        ::close(fd_);
        fd_ = -1;
        LOGI("串口 %s 已关闭", path_.c_str());
    }
}

// ---------------------------------------------------------------------------
// SimDevice
// ---------------------------------------------------------------------------
SimDevice::SimDevice(double tick_hz) : tick_hz_(tick_hz) {
    int p[2];
    if (pipe2(p, O_NONBLOCK) != 0) {
        LOGE("pipe2 失败: %s", strerror(errno));
        return;
    }
    rx_fd_ = p[0];
    tx_fd_ = p[1];
    LOGI("simulate 模式: 未打开任何真实串口, 使用虚拟设备 (tick_hz=%.0f)", tick_hz);
}

SimDevice::~SimDevice() { close(); }

void SimDevice::close() {
    if (rx_fd_ >= 0) ::close(rx_fd_);
    if (tx_fd_ >= 0) ::close(tx_fd_);
    rx_fd_ = tx_fd_ = -1;
}

void SimDevice::set_plant_linear(const double lin[4]) {
    for (int i = 0; i < 4; ++i) plant_linear_[i] = lin[i];
}

void SimDevice::inject_frame(const std::vector<uint8_t>& payload) {
    auto frame = proto::pack_mc_frame(payload);
    ssize_t w = ::write(tx_fd_, frame.data(), frame.size());
    if (w < 0 && errno != EAGAIN) {
        LOGW("simulate 注入应答失败: %s", strerror(errno));
    }
}

// 虚拟设备收到控制线程写来的线路帧:
//   - 编码器读(04 01 <port> ...): 按 plant 线速度积分原始编码值, 回 04 01 <port> <int32>
//   - motor4 轮速(01 02 s1..s4): 从 4 个 int8 反推线速度(校验帧格式, 供日志)
//   - 其他透传帧: 原样回显(dev/mode/payload[2] 保持, 应答匹配键不变)
void SimDevice::write(const uint8_t* d, size_t n) {
    proto::FrameParser parser;
    parser.append(d, n);
    std::vector<uint8_t> payload;
    while (parser.next(payload)) {
        if (payload.size() < 3) continue;
        const uint8_t dev = payload[0], mode = payload[1], p2 = payload[2];
        if (dev == 0x04 && mode == 0x01 && p2 >= 1 && p2 <= 4) {
            // 编码器读: 推进虚拟原始编码(每 tick 一次)
            const int idx = p2 - 1;
            // 与里程计换算互为逆过程: raw_delta = -v_linear*dt*(320.44975/0.03)
            double delta = -plant_linear_[idx] / tick_hz_ * (320.44975 / 0.03);
            enc_raw_[idx] += static_cast<int32_t>(delta);
            // 应答 payload = 04 01 <port> <int32 LE>
            std::vector<uint8_t> rep(7);
            rep[0] = 0x04;
            rep[1] = 0x01;
            rep[2] = p2;
            le32_write(rep.data() + 3, enc_raw_[idx]);
            inject_frame(rep);
        } else if (dev == 0x01 && mode == 0x02 && payload.size() == 6) {
            // motor4 轮速帧: 反推线速度(用于 simulate 日志核对)
            double lin[4];
            for (int i = 0; i < 4; ++i) {
                int8_t s = static_cast<int8_t>(payload[2 + i]);
                lin[i] = -static_cast<double>(s) / ((1.0 / kin::WHEEL_RADIUS) * kin::RAD2VIRTUAL);
            }
            LOGI("[sim] motor4 帧 -> 线速度 [%.3f %.3f %.3f %.3f] m/s",
                 lin[0], lin[1], lin[2], lin[3]);
        } else {
            // 透传帧: 原样回显, 便于联调 frame / frame_async
            inject_frame(payload);
        }
    }
}

// ---------------------------------------------------------------------------
IoBase* create_io(bool simulate, const std::string& port, double tick_hz) {
    if (simulate) return new SimDevice(tick_hz);
    auto* tty = new SerialTty(port);
    if (tty->fd() < 0) {
        delete tty;
        return nullptr;
    }
    return tty;
}
