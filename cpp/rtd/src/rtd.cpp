// rtd.cpp —— 实时控制核心实现
#include "rtd.h"
#include "util.h"

#include <poll.h>
#include <unistd.h>
#include <ctime>
#include <cstring>
#include <algorithm>
#include <chrono>

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// timespec 辅助
// ---------------------------------------------------------------------------
namespace {
inline bool ts_leq(const struct timespec& a, const struct timespec& b) {
    return (a.tv_sec < b.tv_sec) ||
           (a.tv_sec == b.tv_sec && a.tv_nsec <= b.tv_nsec);
}
inline void ts_add_ns(struct timespec& t, long ns) {
    t.tv_nsec += ns;
    while (t.tv_nsec >= 1000000000L) {
        t.tv_nsec -= 1000000000L;
        t.tv_sec += 1;
    }
}
}  // namespace

// ---------------------------------------------------------------------------
RtdCore::RtdCore(Options opt) : opt_(opt) {
    last_enc_reply_ = mono_now();  // 编码器看门狗计时起点
}

RtdCore::~RtdCore() {
    if (io_) {
        delete io_;
        io_ = nullptr;
    }
}

bool RtdCore::start() {
    io_ = create_io(opt_.simulate, opt_.port, static_cast<double>(opt_.tick_hz));
    if (!io_) {
        LOGE("IO 初始化失败 (port=%s)", opt_.port.c_str());
        return false;
    }
    zmq_ = std::make_unique<ZmqServer>(this, opt_.cmd_port, opt_.pub_port);
    if (!zmq_->start()) {
        LOGE("ZMQ 启动失败");
        delete io_;
        io_ = nullptr;
        return false;
    }
    try {
        control_thread_ = std::thread(&RtdCore::control_loop, this);
    } catch (const std::exception& e) {
        LOGE("启动控制线程失败: %s", e.what());
        return false;
    }
    return true;
}

void RtdCore::request_stop() { stop_ = true; }

void RtdCore::wait_stop() {
    if (control_thread_.joinable()) control_thread_.join();
    if (zmq_) zmq_->shutdown();
}

// ---------------------------------------------------------------------------
// 控制主循环: 固定 100Hz (可配 --tick-hz), clock_nanosleep TIMER_ABSTIME 绝对节拍
// ---------------------------------------------------------------------------
void RtdCore::control_loop() {
    const double period = 1.0 / static_cast<double>(opt_.tick_hz);
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    // 节拍误差统计: 首个 tick 用 -1 哨兵跳过(其周期≈0, 会把启动耗时误报为误差)
    last_tick_ = -1.0;
    err_window_start_ = mono_now();
    err_window_max_ = 0.0;

    while (!stop_.load()) {
        // 绝对时间节拍, 避免累积漂移
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        const double tick_start = mono_now();

        // 节拍误差统计(滚动 1s 窗口内最大偏差)
        if (last_tick_ >= 0.0) {
            double err_ms = std::fabs(tick_start - last_tick_ - period) * 1000.0;
            if (err_ms > err_window_max_) err_window_max_ = err_ms;
        }
        last_tick_ = tick_start;
        if (tick_start - err_window_start_ >= 1.0) {
            std::lock_guard<std::mutex> lk(m_);
            tick_err_ms_ = err_window_max_;
            err_window_max_ = 0.0;
            err_window_start_ = tick_start;
        }

        // 1) 写编码器读: 4 帧拼一次 write
        {
            auto blob = proto::mc602::encoder_read_all();  // 28 字节 = 4 x 7
            std::vector<uint8_t> line;
            for (int i = 0; i < 4; ++i) {
                std::vector<uint8_t> payload(blob.begin() + i * 7,
                                             blob.begin() + (i + 1) * 7);
                auto f = proto::pack_mc_frame(payload);
                line.insert(line.end(), f.begin(), f.end());
            }
            io_->write(line.data(), line.size());
        }

        // 2) 接收窗口: 收应答 + 间隙发透传帧
        receive_and_dispatch(tick_start + opt_.recv_budget_ms / 1000.0);

        // 3) 里程计积分(有新编码器值则更新, 否则冻结不崩)
        integrate_odometry();

        // 4) 当前模式控制(VELOCITY / POSITION / IDLE)
        {
            std::lock_guard<std::mutex> lk(m_);
            run_mode_control(tick_start);
        }

        // 5) 异步透传应答超时清扫 -> PUB timeout 事件
        sweep_async_subs();

        // 6) 编码器看门狗
        check_encoder_watchdog();

        // 推进节拍; 过冲(追不上)则跳过丢失的 tick, 跳到未来
        ts_add_ns(next, static_cast<long>(period * 1e9));
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        if (ts_leq(next, now_ts)) {
            next = now_ts;
            ts_add_ns(next, static_cast<long>(period * 1e9));
        }
    }

    // 停机: 强制零速 -> 关串口
    {
        const double zero[3] = {0, 0, 0};
        std::lock_guard<std::mutex> lk(m_);
        compute_and_send_speed(zero, true);  // force: 确保电机收到零速
    }
    io_->close();
    LOGI("控制线程退出, 已发送零速并关闭串口");
}

// 接收窗口: poll 串口, 解析帧并路由应答; 控制帧间隙发一条透传帧
void RtdCore::receive_and_dispatch(double budget_end) {
    struct pollfd pfd;
    pfd.fd = io_->fd();
    pfd.events = POLLIN;
    bool sent_tx = false;
    while (mono_now() < budget_end) {
        if (!sent_tx) {
            send_next_tx();  // 透传帧优先级低于控制帧, 在间隙发送
            sent_tx = true;
        }
        pfd.revents = 0;
        int pr = ::poll(&pfd, 1, 1);
        if (pr > 0 && (pfd.revents & POLLIN)) {
            uint8_t tmp[256];
            ssize_t n = ::read(io_->fd(), tmp, sizeof(tmp));
            if (n > 0) parser_.append(tmp, static_cast<size_t>(n));
            std::vector<uint8_t> payload;
            while (parser_.next(payload)) dispatch_reply(payload);
        }
    }
}

void RtdCore::send_next_tx() {
    std::vector<uint8_t> frame;
    {
        std::lock_guard<std::mutex> lk(m_);
        if (txq_.empty()) return;
        frame = std::move(txq_.front());
        txq_.pop_front();
    }
    io_->write(frame.data(), frame.size());
}

// 应答路由: 编码器应答 -> 同步 frame 等待者 -> 异步 frame 订阅者
// 匹配键 (payload[0], payload[1], payload[2]) = (dev, mode, port)。
// 注意: motor4 帧无 port 字节, 其应答(若有)第 3 字节是 s1(首个轮速),
// 匹配键照旧用 payload[2] —— 与 Python 引擎行为一致, 无需特殊处理。
void RtdCore::dispatch_reply(const std::vector<uint8_t>& payload) {
    if (payload.size() < 3) return;
    const uint8_t dev = payload[0];
    const uint8_t mode = payload[1];
    const uint8_t p2 = payload[2];
    // 编码器应答优先(dev=04 mode=01 port=1..4)
    if (dev == 0x04 && mode == 0x01 && p2 >= 1 && p2 <= 4) {
        handle_encoder_reply(payload);
        return;
    }
    std::lock_guard<std::mutex> lk(m_);
    // 同步 frame 等待者(FIFO)
    for (auto it = sync_waiters_.begin(); it != sync_waiters_.end(); ++it) {
        auto& w = *it;
        if (w->dev == dev && w->mode == mode && w->port == p2) {
            w->ok = true;
            w->payload = payload;
            w->done = true;
            sync_waiters_.erase(it);
            w->cv.notify_all();
            return;
        }
    }
    // 异步 frame 订阅者
    const double now = mono_now();
    for (auto it = async_subs_.begin(); it != async_subs_.end(); ++it) {
        if (it->dev == dev && it->mode == mode && it->port == p2 &&
            now <= it->deadline) {
            json ev = {{"evt", "reply"}, {"seq", it->seq},
                       {"payload", hex_encode(payload)}};
            push_pub_event(ev.dump());
            async_subs_.erase(it);
            return;
        }
    }
}

void RtdCore::handle_encoder_reply(const std::vector<uint8_t>& payload) {
    if (payload.size() < 7) return;
    const uint8_t port = payload[2];
    const int32_t raw = le32_read(payload.data() + 3);
    std::lock_guard<std::mutex> lk(m_);
    enc_cur_[port - 1] = raw;
    fresh_mask_ |= static_cast<uint8_t>(1u << (port - 1));
    last_enc_reply_ = mono_now();
}

// 里程计积分: 4 路编码器本轮都有新值才积分, 否则冻结(不崩)
void RtdCore::integrate_odometry() {
    std::lock_guard<std::mutex> lk(m_);
    if (fresh_mask_ != 0x0F) return;
    double d[4];
    for (int p = 0; p < 4; ++p) {
        if (have_prev_[p]) {
            const int32_t delta = enc_cur_[p] - enc_prev_[p];
            d[p] = kin::enc_delta_to_linear(delta);  // 取负在换算里(见 kinematics.h)
        } else {
            d[p] = 0.0;  // 首次读数: 仅建立基线
        }
        enc_prev_[p] = enc_cur_[p];
        have_prev_[p] = true;
    }
    double cd[3];
    kin::forward_kinematics(d, cd);
    odom_.update(cd);
    fresh_mask_ = 0;
}

void RtdCore::run_mode_control(double now) {
    switch (mode_) {
        case Mode::IDLE: {
            const double zero[3] = {0, 0, 0};
            compute_and_send_speed(zero, false);
            break;
        }
        case Mode::VELOCITY: {
            if (now - last_vel_cmd_ > opt_.vel_watchdog_s) {
                // 看门狗: 最近 vel 命令超 0.5s -> 自动零速
                const double zero[3] = {0, 0, 0};
                compute_and_send_speed(zero, false);
            } else {
                compute_and_send_speed(vel_target_, false);
            }
            break;
        }
        case Mode::POSITION: {
            step_goto(now);
            break;
        }
    }
}

// 逆解 -> 轮速换算 -> 有变化则下发 motor4 帧
// force=true 时强制下发(用于停机/模式切换确保电机收到)
void RtdCore::compute_and_send_speed(const double car_v[3], bool force) {
    double wl[4];
    kin::inverse_kinematics(car_v, wl);
    if (opt_.simulate) {
        static_cast<SimDevice*>(io_)->set_plant_linear(wl);
    }
    int8_t wire[4];
    kin::wheel_linear_to_wire(wl, wire);
    if (force || !wire_valid_ ||
        memcmp(wire, last_wire_, sizeof(wire)) != 0) {
        memcpy(last_wire_, wire, sizeof(wire));
        wire_valid_ = true;
        auto payload = proto::mc602::motor4_speed_payload(wire);
        auto frame = proto::pack_mc_frame(payload);
        io_->write(frame.data(), frame.size());
    }
}

// ---------------------------------------------------------------------------
// move_to_position 闭环 (复刻 MecanumDriver.move_to_position, 每 tick 一步)
// ---------------------------------------------------------------------------
void RtdCore::start_goto_locked(const double target[3], const double maxv[3],
                                const double tol[3], double timeout) {
    pid_x_.setpoint = target[0];
    pid_x_.set_output_limits(-maxv[0], maxv[0]);
    pid_y_.setpoint = target[1];
    pid_y_.set_output_limits(-maxv[1], maxv[1]);
    // pid_yaw: 只设 output_limits; setpoint 每 tick 由归一化目标角更新(与 Python 一致)
    pid_yaw_.set_output_limits(-maxv[2], maxv[2]);

    memcpy(g_target_, target, sizeof(g_target_));
    memcpy(g_maxv_, maxv, sizeof(g_maxv_));
    memcpy(g_tol_, tol, sizeof(g_tol_));
    g_timeout_ = timeout;
    g_consec_ = 0;
    g_iter_ = 0;
    g_start_ = mono_now();
    goto_active_ = true;
    goto_ok_ = false;
    mode_ = Mode::POSITION;
    LOGI("goto 启动 target=[%.3f %.3f %.3f] max_v=[%.3f %.3f %.3f] timeout=%.1f",
         target[0], target[1], target[2], maxv[0], maxv[1], maxv[2], timeout);
}

void RtdCore::step_goto(double now) {
    if (!goto_active_) {
        const double zero[3] = {0, 0, 0};
        compute_and_send_speed(zero, false);
        return;
    }
    // 1) 超时或迭代数 > 1000 -> 结束(ok=false), 发零速
    if (now - g_start_ > g_timeout_) {
        LOGW("goto 超时(%.1fs), 停车", g_timeout_);
        end_goto(false, now);
        return;
    }
    g_iter_ += 1;
    if (g_iter_ > 1000) {
        LOGW("goto 迭代超限(>1000), 停车");
        end_goto(false, now);
        return;
    }
    // 2) 航向 ±π 归一化: 取与当前角最接近的等价目标角, 走最短转向
    const double target_theta =
        g_target_[2] + 2.0 * M_PI *
                           std::round((odom_.th - g_target_[2]) / (2.0 * M_PI));
    const double eff[3] = {g_target_[0], g_target_[1], target_theta};
    // 3) 误差判定: 三项全 < tol 则 consecutive++, 否则清零
    const double err[3] = {std::fabs(odom_.x - eff[0]), std::fabs(odom_.y - eff[1]),
                           std::fabs(odom_.th - eff[2])};
    if (err[0] < g_tol_[0] && err[1] < g_tol_[1] && err[2] < g_tol_[2]) {
        g_consec_ += 1;
        if (g_consec_ > 20) {
            LOGI("goto 到位 (%.3f,%.3f,%.3f)", odom_.x, odom_.y, odom_.th);
            end_goto(true, now);
            return;
        }
    } else {
        g_consec_ = 0;
    }
    // 4) PID(注意顺序: pid_x/pid_y 先算, pid_yaw.setpoint 本轮更新后再算, 与 Python 一致)
    const double vx = pid_x_.call(odom_.x);
    const double vy = pid_y_.call(odom_.y);
    pid_yaw_.setpoint = target_theta;
    const double wz = pid_yaw_.call(odom_.th);
    // 5) world -> car 旋转 (wz 不变)
    const double c = std::cos(odom_.th), s = std::sin(odom_.th);
    const double car_v[3] = {vx * c + vy * s, -vx * s + vy * c, wz};
    // 6) 逆解 + 下发
    compute_and_send_speed(car_v, false);
}

void RtdCore::end_goto(bool ok, double now) {
    (void)now;
    goto_active_ = false;
    goto_ok_ = ok;
    mode_ = Mode::IDLE;
    const double zero[3] = {0, 0, 0};
    compute_and_send_speed(zero, false);  // 结束必发零速(Python: 循环后 set_velocity(0,0,0))
}

// ---------------------------------------------------------------------------
// ZMQ 命令处理
// ---------------------------------------------------------------------------
json RtdCore::handle_command(const json& j) {
    if (!j.is_object() || !j.contains("cmd") || !j["cmd"].is_string()) {
        return {{"ok", false}, {"err", "缺少 cmd 字段"}};
    }
    const std::string cmd = j["cmd"].get<std::string>();
    if (cmd == "vel") return cmd_vel(j);
    if (cmd == "goto") return cmd_goto(j);
    if (cmd == "cancel_goto") return cmd_cancel_goto();
    if (cmd == "stop") return cmd_stop();
    if (cmd == "reset") return cmd_reset(j);
    if (cmd == "state") return cmd_state();
    if (cmd == "frame") return cmd_frame(j);
    if (cmd == "frame_async") return cmd_frame_async(j);
    return {{"ok", false}, {"err", "未知命令: " + cmd}};
}

json RtdCore::cmd_vel(const json& j) {
    double v[3] = {0, 0, 0};
    if (j.contains("v") && j["v"].is_array()) {
        const auto& a = j["v"];
        for (int i = 0; i < 3 && i < static_cast<int>(a.size()); ++i) {
            if (a[i].is_number()) v[i] = a[i].get<double>();
        }
    }
    std::lock_guard<std::mutex> lk(m_);
    mode_ = Mode::VELOCITY;
    vel_target_[0] = v[0];
    vel_target_[1] = v[1];
    vel_target_[2] = v[2];
    last_vel_cmd_ = mono_now();  // 喂看门狗
    return {{"ok", true}};
}

json RtdCore::cmd_goto(const json& j) {
    double target[3] = {0, 0, 0};
    if (j.contains("target") && j["target"].is_array()) {
        const auto& a = j["target"];
        for (int i = 0; i < 3 && i < static_cast<int>(a.size()); ++i) {
            if (a[i].is_number()) target[i] = a[i].get<double>();
        }
    }
    double maxv[3] = {0.2, 0.2, M_PI / 3};
    if (j.contains("max_v") && j["max_v"].is_array()) {
        const auto& a = j["max_v"];
        for (int i = 0; i < 3 && i < static_cast<int>(a.size()); ++i) {
            if (a[i].is_number()) maxv[i] = a[i].get<double>();
        }
    }
    double tol[3] = {0.004, 0.004, 0.02};
    if (j.contains("tol") && j["tol"].is_array()) {
        const auto& a = j["tol"];
        for (int i = 0; i < 3 && i < static_cast<int>(a.size()); ++i) {
            if (a[i].is_number()) tol[i] = a[i].get<double>();
        }
    }
    double timeout = 30.0;
    if (j.contains("timeout") && j["timeout"].is_number()) {
        timeout = j["timeout"].get<double>();
    }
    std::lock_guard<std::mutex> lk(m_);
    // duration 可选: max_v[i] = |target[i]-cur[i]|/duration (与 Python 一致)
    if (j.contains("duration") && j["duration"].is_number()) {
        const double dur = j["duration"].get<double>();
        if (dur > 0) {
            const double cur[3] = {odom_.x, odom_.y, odom_.th};
            for (int i = 0; i < 3; ++i) {
                maxv[i] = std::fabs(target[i] - cur[i]) / dur;
            }
        }
    }
    start_goto_locked(target, maxv, tol, timeout);
    return {{"ok", true}};
}

json RtdCore::cmd_cancel_goto() {
    std::lock_guard<std::mutex> lk(m_);
    if (goto_active_) LOGW("goto 被取消");
    goto_active_ = false;
    goto_ok_ = false;
    mode_ = Mode::IDLE;
    return {{"ok", true}};
}

json RtdCore::cmd_stop() {
    std::lock_guard<std::mutex> lk(m_);
    mode_ = Mode::IDLE;
    goto_active_ = false;
    goto_ok_ = false;
    return {{"ok", true}};
}

json RtdCore::cmd_reset(const json& j) {
    std::lock_guard<std::mutex> lk(m_);
    double x, y, z, d;
    const double* nx = nullptr;
    const double* ny = nullptr;
    const double* nz = nullptr;
    const double* nd = nullptr;
    if (j.contains("x") && j["x"].is_number()) {
        x = j["x"].get<double>();
        nx = &x;
    }
    if (j.contains("y") && j["y"].is_number()) {
        y = j["y"].get<double>();
        ny = &y;
    }
    if (j.contains("z") && j["z"].is_number()) {
        z = j["z"].get<double>();
        nz = &z;
    }
    if (j.contains("distance") && j["distance"].is_number()) {
        d = j["distance"].get<double>();
        nd = &d;
    }
    odom_.reset(nx, ny, nz, nd);  // 字段缺省=保持原值(与 Python Odometry.reset 一致)
    return {{"ok", true}};
}

json RtdCore::cmd_state() { return state_json(false); }

json RtdCore::cmd_frame(const json& j) {
    std::vector<uint8_t> payload;
    if (!j.contains("payload") || !j["payload"].is_string() ||
        !hex_decode(j["payload"].get<std::string>(), payload) || payload.empty()) {
        return {{"ok", false}, {"err", "payload 必须是合法 hex 串"}};
    }
    int timeout_ms = 200;
    if (j.contains("timeout_ms") && j["timeout_ms"].is_number()) {
        timeout_ms = j["timeout_ms"].get<int>();
    }
    if (payload.size() < 3) {
        return {{"ok", false}, {"err", "payload 至少 3 字节(dev/mode/port)"}};
    }
    auto frame = proto::pack_mc_frame(payload);
    auto w = std::make_shared<SyncWaiter>();
    w->dev = payload[0];
    w->mode = payload[1];
    w->port = payload[2];
    {
        std::lock_guard<std::mutex> lk(m_);
        txq_.push_back(std::move(frame));
        sync_waiters_.push_back(w);
    }
    // 阻塞等待应答(与 Python get_anwser 语义一致)
    {
        std::unique_lock<std::mutex> lk(m_);
        w->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                       [&] { return w->done || stop_.load(); });
        auto it = std::find(sync_waiters_.begin(), sync_waiters_.end(), w);
        if (it != sync_waiters_.end()) sync_waiters_.erase(it);
        if (w->done) {
            return {{"ok", true}, {"payload", hex_encode(w->payload)}};
        }
        return {{"ok", false}, {"err", "应答超时"}};
    }
}

json RtdCore::cmd_frame_async(const json& j) {
    std::vector<uint8_t> payload;
    if (!j.contains("payload") || !j["payload"].is_string() ||
        !hex_decode(j["payload"].get<std::string>(), payload) || payload.empty()) {
        return {{"ok", false}, {"err", "payload 必须是合法 hex 串"}};
    }
    uint64_t seq = 0;
    if (j.contains("seq") && j["seq"].is_number()) {
        seq = static_cast<uint64_t>(j["seq"].get<uint64_t>());
    }
    int timeout_ms = 200;
    if (j.contains("timeout_ms") && j["timeout_ms"].is_number()) {
        timeout_ms = j["timeout_ms"].get<int>();
    }
    if (payload.size() < 3) {
        return {{"ok", false}, {"err", "payload 至少 3 字节(dev/mode/port)"}};
    }
    auto frame = proto::pack_mc_frame(payload);
    {
        std::lock_guard<std::mutex> lk(m_);
        txq_.push_back(std::move(frame));
        async_subs_.push_back({seq, payload[0], payload[1], payload[2],
                               mono_now() + timeout_ms / 1000.0});
    }
    return {{"ok", true}};
}

// ---------------------------------------------------------------------------
json RtdCore::state_json(bool evt) const {
    std::lock_guard<std::mutex> lk(m_);
    json j;
    j["x"] = odom_.x;
    j["y"] = odom_.y;
    j["th"] = odom_.th;
    j["dist"] = odom_.distance;
    j["mode"] = mode_str();
    j["goto_active"] = goto_active_;
    j["goto_ok"] = goto_ok_;
    j["tick_err_ms"] = tick_err_ms_;
    if (evt) j["evt"] = "state";
    return j;
}

std::string RtdCore::mode_str() const {
    switch (mode_) {
        case Mode::IDLE: return "idle";
        case Mode::VELOCITY: return "velocity";
        case Mode::POSITION: return "position";
    }
    return "idle";
}

bool RtdCore::pop_pub_event(std::string& out) {
    std::lock_guard<std::mutex> lk(m_);
    if (pub_events_.empty()) return false;
    out = std::move(pub_events_.front());
    pub_events_.pop_front();
    return true;
}

// 注意: 调用方必须已持有 m_ (dispatch_reply / sweep_async_subs 都在持锁状态下调用),
// 这里不能再上锁 —— 否则 std::mutex 自递归上锁会死锁(曾踩过这个坑)。
void RtdCore::push_pub_event(const std::string& s) {
    if (pub_events_.size() > 1024) pub_events_.pop_front();  // 防堆积
    pub_events_.push_back(s);
}

void RtdCore::sweep_async_subs() {
    std::lock_guard<std::mutex> lk(m_);
    const double now = mono_now();
    for (auto it = async_subs_.begin(); it != async_subs_.end();) {
        if (now > it->deadline) {
            json ev = {{"evt", "timeout"}, {"seq", it->seq}};
            push_pub_event(ev.dump());
            it = async_subs_.erase(it);
        } else {
            ++it;
        }
    }
}

void RtdCore::check_encoder_watchdog() {
    std::lock_guard<std::mutex> lk(m_);
    const double now = mono_now();
    if (now - last_enc_reply_ > 1.0 && now - last_enc_warn_ > 5.0) {
        LOGW("编码器应答超 1s 未到, 里程计冻结(不崩)");
        last_enc_warn_ = now;
    }
}
