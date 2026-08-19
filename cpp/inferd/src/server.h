// server.h
//
// 单端口 ZMQ REP 服务器: 每个端口一个后台线程 + 一个独立 TrtModel 实例,
// 与 Python 端 infer_back_end.process_demo(每端口一线程 + 独立引擎)对齐。
// 端口先 bind、后加载引擎 —— 加载期间客户端收到 "false"/"[]",
// 与 Python 端 flag_infer_initok 未置位时的行为一致。
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace inferd {

class TrtModel;

class ZmqServer {
public:
    // port: tcp 端口; engine_path: 本端口引擎; tag: 日志前缀(如 "[lane]")
    ZmqServer(int port, std::string engine_path, std::string tag);
    ~ZmqServer();

    ZmqServer(const ZmqServer&) = delete;
    ZmqServer& operator=(const ZmqServer&) = delete;

    // 启动后台线程(bind + 加载 + 服务循环)
    bool start(std::string* err);

    // 请求停止并等待线程退出(SIGINT/SIGTERM 时由 main 调用)
    void stop();

    bool ready() const { return ready_.load(); }

private:
    void run();                    // 线程主体
    std::string handle(const std::string& req);  // 单请求处理(协议分派)
    bool do_raw(const std::string& data, std::vector<float>& out);

    int port_;
    std::string engine_path_;
    std::string tag_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> ready_{false};
    std::thread thread_;
    std::unique_ptr<TrtModel> model_;  // 仅 run() 线程内使用
};

}  // namespace inferd
