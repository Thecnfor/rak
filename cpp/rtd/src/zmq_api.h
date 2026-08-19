#pragma once
// zmq_api.h —— ZMQ 接口层 (REP 命令 / PUB 状态与应答事件)
//
// REP tcp://127.0.0.1:6010  (JSON)
//   {"cmd":"vel","v":[x,y,z]}                    -> {"ok":true}
//   {"cmd":"goto","target":[x,y,th],"max_v":..,"tol":..,"timeout":..} -> {"ok":true}
//   {"cmd":"cancel_goto"} / {"cmd":"stop"}
//   {"cmd":"reset","x":..,"y":..,"z":..,"distance":..}
//   {"cmd":"state"}                              -> {"x","y","th","dist","mode",
//                                                      "goto_active","goto_ok","tick_err_ms"}
//   {"cmd":"frame","payload":"hex","timeout_ms":200} -> {"ok":true,"payload":"hex"} / {"ok":false}
//   {"cmd":"frame_async","payload":"hex","seq":N}    -> {"ok":true}
//
// PUB tcp://127.0.0.1:6011  (50Hz 状态 + 异步应答/超时事件)
//   {"evt":"state", ...同 state}
//   {"evt":"reply","seq":N,"payload":"hex"} / {"evt":"timeout","seq":N}
#include <string>
#include <thread>

class RtdCore;

class ZmqServer {
public:
    ZmqServer(RtdCore* core, int cmd_port, int pub_port);
    ~ZmqServer();

    // 启动 REP 线程 + PUB 线程(不阻塞)
    bool start();
    // 请求退出并 join 线程
    void shutdown();

private:
    void rep_loop();
    void pub_loop();

    RtdCore* core_;
    int cmd_port_;
    int pub_port_;
    std::thread rep_thread_;
    std::thread pub_thread_;
    volatile bool stop_ = false;
};
