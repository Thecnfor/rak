#pragma once
// serial.h —— 串口抽象: 真实 TTY(termios) 与 --simulate 虚拟设备
//
// 真实串口: /dev/ttyUSB0, 1,000,000 波特 8N1 raw 模式
//   - O_RDWR | O_NOCTTY, ioctl TIOCEXCL 独占
//   - CRTSCTS off, ICANON off (cfmakeraw)
// 写线程唯一(控制线程), 全部串口写串行。
#include <cstdint>
#include <string>
#include <vector>

class IoBase {
public:
    virtual ~IoBase() = default;
    virtual int fd() const = 0;               // 可 poll 的读句柄
    virtual void write(const uint8_t* d, size_t n) = 0;
    virtual void close() = 0;
    virtual bool simulate() const = 0;
};

// 真实串口
class SerialTty : public IoBase {
public:
    explicit SerialTty(const std::string& path);
    ~SerialTty() override { close(); }
    int fd() const override { return fd_; }
    void write(const uint8_t* d, size_t n) override;
    void close() override;
    bool simulate() const override { return false; }

private:
    int fd_ = -1;
    std::string path_;
};

// --simulate 虚拟设备: 不碰真实串口。
//   - 写进来的是线路帧(编码器读/轮速/透传), 虚拟设备解析后往管道注入应答帧
//   - 编码器回放: 根据最近一次"下发的轮速"(plant 线速度)积分原始编码值,
//     使里程计在模拟下跟随指令, 可用于联调 vel/goto/reset 闭环
class SimDevice : public IoBase {
public:
    explicit SimDevice(double tick_hz);
    ~SimDevice() override;
    int fd() const override { return rx_fd_; }
    void write(const uint8_t* d, size_t n) override;
    void close() override;
    bool simulate() const override { return true; }

    // 控制线程每 tick 把 4 轮线速度(逆解后、换算前)喂给模拟 plant
    void set_plant_linear(const double lin[4]);

private:
    void inject_frame(const std::vector<uint8_t>& payload);

    int rx_fd_ = -1;      // 控制线程 poll 的读端
    int tx_fd_ = -1;      // 虚拟设备写端
    double tick_hz_ = 100.0;
    double plant_linear_[4] = {0, 0, 0, 0};
    int32_t enc_raw_[4] = {0, 0, 0, 0};
};

// 工厂: simulate 为真返回 SimDevice, 否则打开真实串口(打开失败返回 nullptr)
IoBase* create_io(bool simulate, const std::string& port, double tick_hz);
