#pragma once
// rtd.h —— 实时控制核心: 100Hz 主循环 / 状态机 / 里程计 / PID 闭环 / TX 队列 / 应答路由
#include "serial.h"
#include "pid.h"
#include "kinematics.h"
#include "protocol.h"
#include "zmq_api.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

class ZmqServer;

class RtdCore {
public:
    struct Options {
        std::string port = "/dev/ttyUSB0";
        int tick_hz = 100;
        int cmd_port = 6010;
        int pub_port = 6011;
        bool simulate = false;
        double recv_budget_ms = 3.0;  // 每 tick 编码器应答接收预算
        double vel_watchdog_s = 0.5;  // VELOCITY 看门狗超时
    };

    explicit RtdCore(Options opt);
    ~RtdCore();

    bool start();         // 建 IO + ZMQ, 启动控制线程
    void request_stop();  // 请求停机(信号处理调用)
    void wait_stop();     // join 各线程
    bool ok() const { return io_ != nullptr; }

    // ZMQ REP 线程调用
    nlohmann::json handle_command(const nlohmann::json& j);
    nlohmann::json state_json(bool evt) const;
    bool pop_pub_event(std::string& out);  // PUB 线程调用

private:
    // ---- 控制线程 ----
    void control_loop();
    void receive_and_dispatch(double budget_end);
    void send_next_tx();
    void dispatch_reply(const std::vector<uint8_t>& payload);
    void handle_encoder_reply(const std::vector<uint8_t>& payload);
    void integrate_odometry();
    void run_mode_control(double now);
    void compute_and_send_speed(const double car_v[3], bool force);
    void step_goto(double now);
    void end_goto(bool ok, double now);
    void start_goto_locked(const double target[3], const double maxv[3],
                           const double tol[3], double timeout);
    void sweep_async_subs();
    void check_encoder_watchdog();
    void push_pub_event(const std::string& s);
    std::string mode_str() const;

    // ---- 命令 ----
    nlohmann::json cmd_vel(const nlohmann::json& j);
    nlohmann::json cmd_goto(const nlohmann::json& j);
    nlohmann::json cmd_cancel_goto();
    nlohmann::json cmd_stop();
    nlohmann::json cmd_reset(const nlohmann::json& j);
    nlohmann::json cmd_state();
    nlohmann::json cmd_frame(const nlohmann::json& j);
    nlohmann::json cmd_frame_async(const nlohmann::json& j);

    Options opt_;
    IoBase* io_ = nullptr;
    std::unique_ptr<ZmqServer> zmq_;
    std::atomic<bool> stop_{false};
    std::thread control_thread_;

    mutable std::mutex m_;
    proto::FrameParser parser_;  // 只被控制线程访问(串行)

    // 里程计 / 编码器
    Odometry odom_;
    int32_t enc_cur_[4] = {0, 0, 0, 0};
    int32_t enc_prev_[4] = {0, 0, 0, 0};
    bool have_prev_[4] = {false, false, false, false};
    uint8_t fresh_mask_ = 0;
    double last_enc_reply_ = 0;
    double last_enc_warn_ = 0;

    // 模式
    enum class Mode { IDLE, VELOCITY, POSITION };
    Mode mode_ = Mode::IDLE;
    double vel_target_[3] = {0, 0, 0};
    double last_vel_cmd_ = 0;

    // goto 状态(复刻 MecanumDriver.move_to_position)
    bool goto_active_ = false;
    bool goto_ok_ = false;
    double g_target_[3] = {0, 0, 0};
    double g_maxv_[3] = {0.2, 0.2, M_PI / 3};
    double g_tol_[3] = {0.004, 0.004, 0.02};
    double g_timeout_ = 30.0;
    double g_start_ = 0;
    int g_iter_ = 0;
    int g_consec_ = 0;

    // 三个 PID 实例持久存在(跨多次 goto 状态保留, 与 Python 一致)
    PID pid_x_ = PID(6, 0.3, 0.1, 0.0, -0.6, 0.6);
    PID pid_y_ = PID(8, 0.3, 0.1, 0.0, -0.6, 0.6);
    PID pid_yaw_ = PID(10, 0.2, 0.1, 0.0, -1.5, 1.5);

    // 轮速下发(有变化才发)
    int8_t last_wire_[4] = {0, 0, 0, 0};
    bool wire_valid_ = false;

    // 透传 TX 队列(优先级低于控制帧, 由控制线程在帧间隙发送)
    std::deque<std::vector<uint8_t>> txq_;

    // 同步应答等待(frame 命令, REP 线程阻塞等待, 控制线程应答后唤醒)
    struct SyncWaiter {
        uint8_t dev = 0, mode = 0, port = 0;
        bool done = false;
        bool ok = false;
        std::vector<uint8_t> payload;
        std::condition_variable cv;
    };
    std::deque<std::shared_ptr<SyncWaiter>> sync_waiters_;

    // 异步应答订阅(frame_async 命令)
    struct AsyncSub {
        uint64_t seq = 0;
        uint8_t dev = 0, mode = 0, port = 0;
        double deadline = 0;
    };
    std::vector<AsyncSub> async_subs_;

    // PUB 事件队列(由控制线程推, PUB 线程取)
    std::deque<std::string> pub_events_;

    // 节拍误差统计(ms)
    double tick_err_ms_ = 0;
    double last_tick_ = 0;
    double err_window_start_ = 0;
    double err_window_max_ = 0;
};
