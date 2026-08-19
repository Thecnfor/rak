// engine.cpp
#include "engine.h"

#include <cuda_runtime.h>
#include <NvInferRuntime.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

#include "preprocess.h"

namespace inferd {

namespace {

// TRT 日志: 只输出 WARNING 及以上, 避免刷屏(与 Python 端 trt.Logger(ERROR)
// 近似; 多留 WARNING 便于排查反序列化失败)。
class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity sev, const char* msg) noexcept override {
        if (sev <= Severity::kWARNING) {
            std::cerr << "[TRT] " << msg << "\n";
        }
    }
};

TrtLogger g_logger;

}  // namespace

TrtModel::~TrtModel() { release(); }

void TrtModel::release() {
    if (stream_) cudaStreamDestroy(stream_);
    stream_ = nullptr;
    // TRT 10 已移除 destroy(), 对象统一用 delete 释放(见 NvInferRuntime.h)
    if (context_) delete context_;
    context_ = nullptr;
    if (engine_) delete engine_;
    engine_ = nullptr;
    if (runtime_) delete runtime_;
    runtime_ = nullptr;
    if (input_host_) cudaFreeHost(input_host_);
    if (output_host_) cudaFreeHost(output_host_);
    if (input_device_) cudaFree(input_device_);
    if (output_device_) cudaFree(output_device_);
    input_host_ = output_host_ = nullptr;
    input_device_ = output_device_ = nullptr;
    input_bytes_ = output_bytes_ = 0;
    first_output_name_.clear();
}

bool TrtModel::load(const std::string& engine_path, std::string* err) {
    auto fail = [&](const std::string& msg) {
        if (err) *err = msg;
        return false;
    };

    // 读引擎文件到内存, 一次性 deserialize(引擎字节流, 无独立分配器依赖)
    std::ifstream f(engine_path, std::ios::binary);
    if (!f) return fail("无法打开引擎文件: " + engine_path);
    std::vector<char> data((std::istreambuf_iterator<char>(f)),
                           std::istreambuf_iterator<char>());
    f.close();
    if (data.empty()) return fail("引擎文件为空: " + engine_path);

    runtime_ = nvinfer1::createInferRuntime(g_logger);
    if (!runtime_) return fail("createInferRuntime 失败(TRT 运行时不可用)");
    engine_ = runtime_->deserializeCudaEngine(data.data(), data.size());
    if (!engine_) return fail("deserializeCudaEngine 失败(引擎与当前 TRT 版本不匹配?)");
    context_ = engine_->createExecutionContext();
    if (!context_) return fail("createExecutionContext 失败");
    if (cudaStreamCreate(&stream_) != cudaSuccess) return fail("cudaStreamCreate 失败");

    // 遍历 IO 张量, 记录形状/类型并区分输入输出(TRT10 无 binding 索引,
    // 全部按名字操作)
    std::vector<std::string> input_names, output_names;
    const int nb = engine_->getNbIOTensors();
    for (int i = 0; i < nb; ++i) {
        const char* nm = engine_->getIOTensorName(i);
        const nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(nm);
        const nvinfer1::Dims dims = engine_->getTensorShape(nm);
        const nvinfer1::DataType dt = engine_->getTensorDataType(nm);
        std::ostringstream ss;
        ss << "[engine] tensor " << nm
           << " mode=" << (mode == nvinfer1::TensorIOMode::kINPUT ? "input" : "output")
           << " shape=[";
        for (int d = 0; d < dims.nbDims; ++d) {
            if (d) ss << ",";
            ss << dims.d[d];
        }
        ss << "] dtype="
           << (dt == nvinfer1::DataType::kFLOAT ? "fp32"
               : dt == nvinfer1::DataType::kHALF ? "fp16"
                                                 : "other");
        std::cerr << ss.str() << "\n";
        if (mode == nvinfer1::TensorIOMode::kINPUT)
            input_names.emplace_back(nm);
        else
            output_names.emplace_back(nm);
    }
    if (input_names.empty()) return fail("引擎没有输入张量");
    if (output_names.empty()) return fail("引擎没有输出张量");

    // --- 输入: 期望恰好一个, "inputs", 1x3x128x128 fp32 ---
    if (input_names.size() != 1) {
        return fail("引擎输入张量不止一个, 本程序仅支持单输入小 CNN");
    }
    {
        const std::string& iname = input_names[0];
        if (iname != "inputs") {
            std::cerr << "[warn] 输入张量名不是 'inputs' 而是 '" << iname << "'\n";
        }
        const nvinfer1::Dims dims = engine_->getTensorShape(iname.c_str());
        int64_t elems = 1;
        bool dynamic = false;
        for (int d = 0; d < dims.nbDims; ++d) {
            if (dims.d[d] < 0) {
                dynamic = true;
                break;
            }
            elems *= dims.d[d];
        }
        if (dynamic) {
            // 动态输入(不该出现, 但宽容处理): 按固定 3*128*128 分配
            std::cerr << "[warn] 输入 '" << iname << "' 含动态维, 按 1x3x128x128 分配\n";
            elems = static_cast<int64_t>(kInputFloats);
        } else if (elems != static_cast<int64_t>(kInputFloats)) {
            return fail("输入张量形状不是 1x3x128x128(" + std::to_string(elems) +
                        " 元素), 与预处理不符");
        }
        if (engine_->getTensorDataType(iname.c_str()) != nvinfer1::DataType::kFLOAT) {
            return fail("输入张量不是 fp32, 本程序仅支持 fp32 输入");
        }
        input_bytes_ = static_cast<size_t>(elems) * sizeof(float);
        if (cudaMalloc(&input_device_, input_bytes_) != cudaSuccess ||
            cudaMallocHost(&input_host_, input_bytes_) != cudaSuccess) {
            return fail("输入内存分配失败(设备或 pinned 主机)");
        }
        // 输入形状固定, 地址一次绑定即可(与 Python 每次 set_tensor_address
        // 等价, 地址从未改变)
        context_->setInputTensorAddress(iname.c_str(), input_device_);
    }

    // --- 输出: 只序列化第一个(与 Python next(iter(out)) 一致), 其余仅记录 ---
    {
        const std::string& oname = output_names[0];
        const nvinfer1::Dims dims = engine_->getTensorShape(oname.c_str());
        int64_t elems = 1;
        for (int d = 0; d < dims.nbDims; ++d) {
            if (dims.d[d] < 0) {
                return fail("输出张量含动态维, 本程序仅支持静态输出的 lane/correction");
            }
            elems *= dims.d[d];
        }
        if (engine_->getTensorDataType(oname.c_str()) != nvinfer1::DataType::kFLOAT) {
            return fail("输出张量不是 fp32");
        }
        output_bytes_ = static_cast<size_t>(elems) * sizeof(float);
        if (cudaMalloc(&output_device_, output_bytes_) != cudaSuccess ||
            cudaMallocHost(&output_host_, output_bytes_) != cudaSuccess) {
            return fail("输出内存分配失败(设备或 pinned 主机)");
        }
        first_output_name_ = oname;
        context_->setOutputTensorAddress(oname.c_str(), output_device_);
        if (output_names.size() > 1) {
            std::cerr << "[warn] 共有 " << output_names.size() << " 个输出, "
                      << "仅序列化第一个 '" << oname << "'(与 Python 端一致)\n";
        }
    }

    std::cerr << "[engine] " << engine_path << " 加载完成, 输出 '" << first_output_name_
              << "' " << (output_bytes_ / sizeof(float)) << " 个 float\n";
    return true;
}

bool TrtModel::infer(const uint8_t* bgr, int h, int w, std::vector<float>& out) {
    if (!loaded() || !context_ || !input_host_ || !output_host_) return false;

    // 预处理直接写入 pinned 缓冲, 省去一次主机拷贝
    float* host_in = static_cast<float*>(input_host_);
    if (!preprocess_bgr(bgr, h, w, host_in)) return false;

    // 同一 CUDA 流上按序: H2D -> 推理 -> D2H, 末尾一次同步即可
    // (Python 端 execute_async_v3 + stream_sync 也是同一流串行)
    if (cudaMemcpyAsync(input_device_, host_in, input_bytes_,
                        cudaMemcpyHostToDevice, stream_) != cudaSuccess)
        return false;
    if (!context_->enqueueV3(stream_)) return false;
    if (cudaMemcpyAsync(output_host_, output_device_, output_bytes_,
                        cudaMemcpyDeviceToHost, stream_) != cudaSuccess)
        return false;
    if (cudaStreamSynchronize(stream_) != cudaSuccess) return false;

    const size_t n = output_bytes_ / sizeof(float);
    const float* host_out = static_cast<const float*>(output_host_);
    out.assign(host_out, host_out + n);
    return true;
}

}  // namespace inferd
