// main.cpp —— rtd 入口: CLI 解析 / 信号处理 / 生命周期
#include "rtd.h"
#include "kinematics.h"
#include "util.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};

static void on_signal(int) { g_stop = true; }

static void usage(const char* prog) {
    fprintf(stderr,
            "用法: %s [选项]\n"
            "  --port PATH     串口设备 (默认 /dev/ttyUSB0)\n"
            "  --tick-hz N     控制频率 Hz (默认 100)\n"
            "  --cmd-port N    ZMQ REP 端口 (默认 6010)\n"
            "  --pub-port N    ZMQ PUB 端口 (默认 6011)\n"
            "  --simulate      不开串口, 用虚拟设备联调(安全, 不碰真实硬件)\n"
            "  --recv-budget MS 每 tick 编码器应答接收预算 ms (默认 3)\n"
            "  --help          显示本帮助\n",
            prog);
}

int main(int argc, char** argv) {
    RtdCore::Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next_val = [&](std::string& out) -> bool {
            if (i + 1 < argc) {
                out = argv[++i];
                return true;
            }
            return false;
        };
        std::string v;
        if (a == "--port") {
            if (!next_val(v)) { usage(argv[0]); return 2; }
            opt.port = v;
        } else if (a == "--tick-hz") {
            if (!next_val(v)) { usage(argv[0]); return 2; }
            opt.tick_hz = std::stoi(v);
        } else if (a == "--cmd-port") {
            if (!next_val(v)) { usage(argv[0]); return 2; }
            opt.cmd_port = std::stoi(v);
        } else if (a == "--pub-port") {
            if (!next_val(v)) { usage(argv[0]); return 2; }
            opt.pub_port = std::stoi(v);
        } else if (a == "--recv-budget") {
            if (!next_val(v)) { usage(argv[0]); return 2; }
            opt.recv_budget_ms = std::stod(v);
        } else if (a == "--simulate") {
            opt.simulate = true;
        } else if (a == "--help" || a == "-h") {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "未知参数: %s\n", a.c_str());
            usage(argv[0]);
            return 2;
        }
    }

    // 构建自检: 验证轮速换算/帧字节与 Python 复算一致
    kinematics_selftest();

    RtdCore core(opt);
    if (!core.start()) {
        LOGE("rtd 启动失败");
        return 1;
    }
    LOGI("rtd 已启动: tick=%dHz, port=%s, cmd=6010, pub=6011, simulate=%s",
         opt.tick_hz, opt.port.c_str(), opt.simulate ? "on" : "off");
    LOGI("警告: 当前%s连接真实串口。上车部署请用 systemd 管理并确认 Python 进程已停止",
         opt.simulate ? "未" : "将");

    // SIGINT / SIGTERM -> 零速 -> 关串口退出
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOGI("收到退出信号, 正在停机: 零速 -> 关串口");
    core.request_stop();
    core.wait_stop();
    LOGI("rtd 已退出");
    return 0;
}
