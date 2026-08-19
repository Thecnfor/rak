// main.cpp
//
// C++ TensorRT 推理守护进程: 接管 lane(5001) / correction(5003) 两个端口,
// task(5002) 仍由原 Python 后端(infer_back_end.py)接管。
// 两个端口各一个后台线程 + 独立引擎实例, 参见 server.h / engine.h。
//
// 部署方式: 复制 scripts/infer-cpp.service 到 /etc/systemd/system/ 并
// systemctl enable --now(详见 README「部署」与「切换与回退」)。
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "engine.h"
#include "preprocess.h"
#include "server.h"

namespace {

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

void print_usage(const char* argv0) {
    std::cerr
        << "用法: " << argv0 << " [选项]\n"
        << "  --lane-engine PATH          lane 引擎路径\n"
        << "                              默认 /home/xrak/rak/trt_engines/lane_fp16.engine\n"
        << "  --correction-engine PATH    correction 引擎路径\n"
        << "                              默认 /home/xrak/rak/trt_engines/correction_fp16.engine\n"
        << "  --lane-port PORT            lane 端口(默认 5001)\n"
        << "  --correction-port PORT      correction 端口(默认 5003)\n"
        << "  --selftest                  加载引擎跑一张全零 128x128 图, 打印输出后退出(不 bind 端口)\n"
        << "  --help, -h                  显示本帮助\n";
}

struct Options {
    std::string lane_engine = "/home/xrak/rak/trt_engines/lane_fp16.engine";
    std::string corr_engine = "/home/xrak/rak/trt_engines/correction_fp16.engine";
    int lane_port = 5001;
    int corr_port = 5003;
    bool selftest = false;
};

// 手工解析命令行(不引入 CLI 库, 参数很少)
bool parse_args(int argc, char** argv, Options* opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        // 取下一个参数值, 不存在则报错退出
        const auto next = [&](std::string* dst) {
            if (i + 1 >= argc) {
                std::cerr << "缺少参数值: " << a << "\n";
                return false;
            }
            *dst = argv[++i];
            return true;
        };
        if (a == "--lane-engine") {
            if (!next(&opt->lane_engine)) return false;
        } else if (a == "--correction-engine") {
            if (!next(&opt->corr_engine)) return false;
        } else if (a == "--lane-port") {
            std::string v;
            if (!next(&v)) return false;
            opt->lane_port = std::stoi(v);
        } else if (a == "--correction-port") {
            std::string v;
            if (!next(&v)) return false;
            opt->corr_port = std::stoi(v);
        } else if (a == "--selftest") {
            opt->selftest = true;
        } else if (a == "--help" || a == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            std::cerr << "未知参数: " << a << "\n";
            return false;
        }
    }
    return true;
}

// 自检模式: 不 bind 端口, 只加载两个引擎并跑一张全零 128x128 图
// (归一化后全 -1.0, 与 Python 端预热空白图一致), 打印输出后退出。
int run_selftest(const Options& opt) {
    const std::vector<std::pair<std::string, std::string>> jobs = {
        {"lane", opt.lane_engine},
        {"correction", opt.corr_engine},
    };
    int fail = 0;
    for (const auto& [tag, path] : jobs) {
        inferd::TrtModel m;
        std::string err;
        if (!m.load(path, &err)) {
            std::cerr << "[selftest] " << tag << " 加载失败: " << err << "\n";
            ++fail;
            continue;
        }
        std::vector<uint8_t> zeros(static_cast<size_t>(inferd::kInputH) *
                                       inferd::kInputW * inferd::kInputC,
                                   0);
        std::vector<float> out;
        if (!m.infer(zeros.data(), inferd::kInputH, inferd::kInputW, out)) {
            std::cerr << "[selftest] " << tag << " 推理失败\n";
            ++fail;
            continue;
        }
        std::cerr << "[selftest] " << tag << " 输出(" << out.size() << "): [";
        for (size_t i = 0; i < out.size(); ++i) {
            if (i) std::cerr << ", ";
            std::cerr << out[i];
        }
        std::cerr << "]\n";
    }
    return fail == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, &opt)) {
        print_usage(argv[0]);
        return 2;
    }

    if (opt.selftest) {
        return run_selftest(opt);
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::cerr << "[inferd] C++ TRT 推理守护进程启动 "
              << "(lane:5001 correction:5003, task:5002 仍由 Python 后端接管)\n";

    inferd::ZmqServer lane(opt.lane_port, opt.lane_engine, "[lane]");
    inferd::ZmqServer corr(opt.corr_port, opt.corr_engine, "[correction]");

    std::string err;
    if (!lane.start(&err)) {
        std::cerr << "[inferd] lane 线程启动失败: " << err << "\n";
        return 1;
    }
    if (!corr.start(&err)) {
        std::cerr << "[inferd] correction 线程启动失败: " << err << "\n";
        lane.stop();
        return 1;
    }

    // 主线程挂起, 等待信号(守护进程由 systemd 管理, 无需 daemonize)
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cerr << "[inferd] 收到退出信号, 正在关闭...\n";
    lane.stop();
    corr.stop();
    std::cerr << "[inferd] 已退出\n";
    return 0;
}
