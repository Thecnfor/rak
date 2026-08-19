// engine.h
//
// TensorRT 引擎封装, 逐语义复刻 Python 端 TrtEngine(TrtLaneInfer/TrtCorrectionInfer
// 共用的底层)。串行流程: 预处理到 pinned 主机缓冲 -> cudaMemcpyAsync H2D ->
// enqueueV3 -> cudaMemcpyAsync D2H -> 同步, 与 Python 的
// memcpy_htod / execute_async_v3 / stream_sync 一一对应。
//
// 每个端口一个实例, 单线程内使用, 不做跨线程共享(与 Python 端
// 每端口独立 TrtEngine 的线程模型一致)。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

// 仅前置声明, 不把 NvInferRuntime.h / cuda_runtime.h 引入头文件,
// 缩小编译面; 完整类型只在 engine.cpp 里使用。
struct CUstream_st;
namespace nvinfer1 {
class IRuntime;
class ICudaEngine;
class IExecutionContext;
}  // namespace nvinfer1

namespace inferd {

class TrtModel {
public:
    TrtModel() = default;
    ~TrtModel();

    // 禁止拷贝(持有 CUDA/TRT 资源), 只允许移动/析构
    TrtModel(const TrtModel&) = delete;
    TrtModel& operator=(const TrtModel&) = delete;

    // 从 .engine 反序列化并创建执行上下文, 分配设备/主机 pinned 内存,
    // 一次性绑定 IO 张量地址。失败返回 false 并给出中文错误信息。
    bool load(const std::string& engine_path, std::string* err);

    // 对 BGR 帧(连续 h*w*3 字节)推理; 结果写入 out(第一个输出张量的
    // 全部 float, 与 Python 端 output_data.tolist() 一致)。
    // 返回 false 时 out 内容无效。
    bool infer(const uint8_t* bgr, int h, int w, std::vector<float>& out);

    bool loaded() const { return engine_ != nullptr; }

private:
    // 资源全部在 load() 中分配, 析构统一释放
    void release();

    CUstream_st* stream_ = nullptr;  // cudaStream_t
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;

    void* input_device_ = nullptr;   // 设备端输入缓冲(1x3x128x128 fp32)
    void* input_host_ = nullptr;     // pinned 主机输入缓冲, 预处理直写
    void* output_device_ = nullptr;  // 设备端输出缓冲(第一个输出)
    void* output_host_ = nullptr;    // pinned 主机输出缓冲
    size_t input_bytes_ = 0;
    size_t output_bytes_ = 0;
    std::string first_output_name_;  // 仅序列化第一个输出(与 Python 一致)
};

}  // namespace inferd
