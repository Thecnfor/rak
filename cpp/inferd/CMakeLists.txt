cmake_minimum_required(VERSION 3.13)
project(inferd CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 只链接现成库, 不启 nvcc: 全工程无 .cu, 只用 cuda_runtime.h 主机 API
# (cudaMalloc/cudaMemcpyAsync/cudaStreamXxx), 见 README「依赖」一节。
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# ---------------------------------------------------------------------------
# 平台库定位 (Jetson aarch64 / TRT 10.3 / CUDA 12.6)
# 各库已确认存在于:
#   nvinfer: /usr/lib/aarch64-linux-gnu/libnvinfer.so
#           头文件 /usr/include/aarch64-linux-gnu/NvInferRuntime.h
#   cudart:  /usr/local/cuda-12.6/targets/aarch64-linux/lib/libcudart.so
#           头文件 /usr/local/cuda-12.6/targets/aarch64-linux/include/cuda_runtime.h
#   zmq:     cppzmq 头 /usr/include/zmq.hpp + /usr/lib/aarch64-linux-gnu/libzmq.so
# ---------------------------------------------------------------------------
find_library(NVINFER_LIB NAMES nvinfer
  PATHS /usr/lib/aarch64-linux-gnu /usr/local/cuda/lib64)
if(NOT NVINFER_LIB)
  message(FATAL_ERROR "未找到 libnvinfer (TRT 10.3), 请检查 TensorRT 安装")
endif()

find_library(CUDART_LIB NAMES cudart
  PATHS /usr/local/cuda-12.6/targets/aarch64-linux/lib
        /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu /usr/lib)
if(NOT CUDART_LIB)
  message(FATAL_ERROR "未找到 libcudart (CUDA 12.6)")
endif()

find_library(ZMQ_LIB NAMES zmq
  PATHS /usr/lib/aarch64-linux-gnu /usr/local/lib)
if(NOT ZMQ_LIB)
  message(FATAL_ERROR "未找到 libzmq")
endif()

add_executable(inferd
  src/main.cpp
  src/engine.cpp
  src/preprocess.cpp
  src/server.cpp
)

target_include_directories(inferd PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  /usr/include/aarch64-linux-gnu              # NvInferRuntime.h / NvInfer.h
  /usr/local/cuda-12.6/targets/aarch64-linux/include  # cuda_runtime.h
  /usr/include                                # zmq.hpp / nlohmann/json.hpp
)

target_link_libraries(inferd PRIVATE
  ${NVINFER_LIB}
  ${CUDART_LIB}
  ${ZMQ_LIB}
)
