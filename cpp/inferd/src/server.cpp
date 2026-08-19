// server.cpp
#include "server.h"

#include <zmq.hpp>

#include <cstring>
#include <iostream>
#include <nlohmann/json.hpp>

#include "engine.h"

namespace inferd {

namespace {

// 帧体长度校验失败 / 协议无效时的统一回复("[]" 与 Python 端空列表一致)
const char* const kEmptyList = "[]";

// float 向量 -> JSON 数组字符串, 与 Python json.dumps(list) 语义等价。
// 用 nlohmann::json 保证最简表示与特殊值(nan/inf -> null)安全性。
std::string to_json_array(const std::vector<float>& v) {
    nlohmann::json arr = nlohmann::json::array();
    for (float x : v) arr.push_back(x);
    return arr.dump();
}

}  // namespace

ZmqServer::ZmqServer(int port, std::string engine_path, std::string tag)
    : port_(port),
      engine_path_(std::move(engine_path)),
      tag_(std::move(tag)) {}

ZmqServer::~ZmqServer() { stop(); }

bool ZmqServer::start(std::string* err) {
    try {
        thread_ = std::thread(&ZmqServer::run, this);
    } catch (const std::system_error& e) {
        if (err) *err = e.what();
        return false;
    }
    return true;
}

void ZmqServer::stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
}

void ZmqServer::run() {
    // 每端口独立 ZMQ 上下文, 避免共享 context 的线程安全问题;
    // RCVTIMEO=500ms 让阻塞的 recv 周期性返回, 使退出不依赖 ETERM 中断。
    zmq::context_t ctx(1);
    zmq::socket_t sock(ctx, zmq::socket_type::rep);
    sock.set(zmq::sockopt::linger, 0);
    sock.set(zmq::sockopt::rcvtimeo, 500);
    try {
        sock.bind("tcp://127.0.0.1:" + std::to_string(port_));
    } catch (const zmq::error_t& e) {
        // 端口被占用(如 Python 后端仍在跑 5001/5003)时无法启动, 线程退出
        std::cerr << tag_ << " bind 失败(端口 " << port_ << " 被占用?): "
                  << e.what() << "\n";
        return;
    }

    // 端口先绑、引擎后载: 加载期间请求回 "false"/"[]"(与 Python 语义一致)
    std::unique_ptr<TrtModel> model(new TrtModel());
    std::string lerr;
    const bool loaded = model->load(engine_path_, &lerr);
    if (!loaded) std::cerr << tag_ << " 引擎加载失败: " << lerr << "\n";
    model_ = std::move(model);
    ready_.store(loaded);
    std::cerr << tag_ << " 监听 tcp://127.0.0.1:" << port_ << ", 引擎"
              << (loaded ? "就绪" : "未加载(将持续回 false/[])") << "\n";

    while (!stop_.load()) {
        zmq::message_t req;
        // cppzmq 的 recv 在 RCVTIMEO 到期时返回 空 optional(不抛异常!),
        // 只有真错误才抛 zmq::error_t。空 optional = 无请求, 直接跳过,
        // 否则会对着没收到请求的 REP socket 调 send -> EFSM 死循环重建。
        std::optional<std::size_t> received;
        try {
            received = sock.recv(req, zmq::recv_flags::none);
        } catch (const zmq::error_t& e) {
            // ETERM(仅 context.shutdown 触发, 本程序不用)则直接退出
            if (stop_.load() || zmq_errno() == ETERM) break;
            continue;
        }
        if (!received.has_value()) {
            // RCVTIMEO 到期(EAGAIN)是正常轮询, 继续以检查退出标志
            continue;
        }

        // REP 要求一次请求必须 recv 全部分片后才允许 send;
        // 只取第一帧, 丢弃多余 multipart, 维持一对一状态机
        std::string data(static_cast<const char*>(req.data()), req.size());
        while (true) {
            bool more = false;
            try {
                more = sock.get(zmq::sockopt::rcvmore);
            } catch (...) {
                break;
            }
            if (!more) break;
            zmq::message_t extra;
            try {
                (void)sock.recv(extra, zmq::recv_flags::none);
            } catch (...) {
                break;
            }
        }

        const std::string reply = handle(data);
        try {
            sock.send(zmq::buffer(reply), zmq::send_flags::none);
        } catch (const zmq::error_t& e) {
            // 发送失败(客户端提前断开): REP 状态机可能失步, 重建 socket
            std::cerr << tag_ << " send 失败(" << e.what() << "), 重建 socket\n";
            try {
                sock.close();
            } catch (...) {
            }
            try {
                sock = zmq::socket_t(ctx, zmq::socket_type::rep);
                sock.set(zmq::sockopt::linger, 0);
                sock.set(zmq::sockopt::rcvtimeo, 500);
                sock.bind("tcp://127.0.0.1:" + std::to_string(port_));
            } catch (const zmq::error_t& be) {
                std::cerr << tag_ << " 重建 bind 失败: " << be.what() << "\n";
            }
        }
    }
    try {
        sock.close();
    } catch (...) {
    }
}

std::string ZmqServer::handle(const std::string& data) {
    // 协议与 Python infer_back_end.process_demo 逐语义兼容:
    //   b"ATATA"            -> json bool, 引擎就绪与否
    //   b"rawi"+<II h,w>+BGR -> json float 数组
    //   b"image"+JPEG       -> 旧协议, 不支持, 回 "[]"
    //   其余                -> "[]"
    if (data.size() >= 5 && data.compare(0, 5, "ATATA") == 0) {
        return ready_.load() ? "true" : "false";
    }

    if (data.size() >= 4 && data.compare(0, 4, "rawi") == 0) {
        if (!ready_.load() || !model_ || !model_->loaded()) return kEmptyList;
        std::vector<float> out;
        if (!do_raw(data, out)) return kEmptyList;
        return to_json_array(out);
    }

    if (data.size() >= 5 && data.compare(0, 5, "image") == 0) {
        // 旧 JPEG 协议: C++ 侧不引入 JPEG 解码库, 按需求直接回 "[]"
        return kEmptyList;
    }

    return kEmptyList;
}

bool ZmqServer::do_raw(const std::string& data, std::vector<float>& out) {
    // b"rawi"(4) + <II 小端 h, w>(8) + BGR 连续像素
    if (data.size() < 12) return false;
    // 小端 uint32: aarch64/x86 均为小端, memcpy 直接取(避免字节序转换)
    uint32_t h = 0, w = 0;
    std::memcpy(&h, data.data() + 4, sizeof(h));
    std::memcpy(&w, data.data() + 8, sizeof(w));
    const size_t body_len = data.size() - 12;
    // 形状校验: h*w*3 必须与帧体长度一致, 否则回 "[]"(Python 端
    // np.frombuffer().reshape 会抛异常, 这里更健壮)
    if (h == 0 || w == 0) return false;
    if (static_cast<size_t>(h) * w * 3 != body_len) return false;
    return model_->infer(reinterpret_cast<const uint8_t*>(data.data()) + 12,
                         static_cast<int>(h), static_cast<int>(w), out);
}

}  // namespace inferd
