// zmq_api.cpp —— ZMQ REP/PUB 线程实现
#include "zmq_api.h"
#include "rtd.h"
#include "util.h"

#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstring>

using json = nlohmann::json;

ZmqServer::ZmqServer(RtdCore* core, int cmd_port, int pub_port)
    : core_(core), cmd_port_(cmd_port), pub_port_(pub_port) {}

ZmqServer::~ZmqServer() { shutdown(); }

bool ZmqServer::start() {
    try {
        rep_thread_ = std::thread(&ZmqServer::rep_loop, this);
        pub_thread_ = std::thread(&ZmqServer::pub_loop, this);
    } catch (const std::exception& e) {
        LOGE("启动 ZMQ 线程失败: %s", e.what());
        return false;
    }
    return true;
}

void ZmqServer::shutdown() {
    if (stop_) return;
    stop_ = true;
    // REP 线程 recv 设置了 100ms 接收超时, 会在超时后检查 stop_ 退出
    if (rep_thread_.joinable()) rep_thread_.join();
    if (pub_thread_.joinable()) pub_thread_.join();
}

// REP 线程: 独占 REP socket, 循环收命令/回 JSON
void ZmqServer::rep_loop() {
    zmq::context_t ctx(1);
    zmq::socket_t sock(ctx, zmq::socket_type::rep);
    const std::string addr = "tcp://127.0.0.1:" + std::to_string(cmd_port_);
    try {
        sock.bind(addr);
    } catch (const zmq::error_t& e) {
        LOGE("REP bind %s 失败: %s", addr.c_str(), e.what());
        return;
    }
    // 接收超时, 便于停机时检查 stop_ 退出
    sock.set(zmq::sockopt::rcvtimeo, 100);
    LOGI("REP 监听 %s", addr.c_str());

    while (!stop_) {
        zmq::message_t req;
        try {
            if (!sock.recv(req, zmq::recv_flags::none)) continue;  // 超时
        } catch (const zmq::error_t&) {
            continue;  // 超时/中断
        }
        std::string in(static_cast<char*>(req.data()), req.size());
        json rep;
        try {
            const json j = json::parse(in);
            rep = core_->handle_command(j);
        } catch (const std::exception& e) {
            LOGW("命令解析失败: %s", e.what());
            rep = {{"ok", false}, {"err", std::string(e.what())}};
        }
        std::string out = rep.dump();
        sock.send(zmq::buffer(out), zmq::send_flags::none);
    }
    LOGI("REP 线程退出");
}

// PUB 线程: 独占 PUB socket; 每 20ms 推一条 50Hz 状态, 事件(应答/超时)即时清空
void ZmqServer::pub_loop() {
    zmq::context_t ctx(1);
    zmq::socket_t sock(ctx, zmq::socket_type::pub);
    const std::string addr = "tcp://127.0.0.1:" + std::to_string(pub_port_);
    try {
        sock.bind(addr);
    } catch (const zmq::error_t& e) {
        LOGE("PUB bind %s 失败: %s", addr.c_str(), e.what());
        return;
    }
    LOGI("PUB 监听 %s (50Hz 状态)", addr.c_str());

    const auto period = std::chrono::milliseconds(20);
    auto next = std::chrono::steady_clock::now() + period;
    while (!stop_) {
        // 事件队列先清空(低延迟推送 reply/timeout)
        std::string evt;
        while (core_->pop_pub_event(evt)) {
            sock.send(zmq::buffer(evt), zmq::send_flags::none);
        }
        // 50Hz 状态
        json st = core_->state_json(true);
        std::string out = st.dump();
        sock.send(zmq::buffer(out), zmq::send_flags::none);

        std::this_thread::sleep_until(next);
        next += period;
        if (std::chrono::steady_clock::now() > next) {
            next = std::chrono::steady_clock::now() + period;
        }
    }
    LOGI("PUB 线程退出");
}
