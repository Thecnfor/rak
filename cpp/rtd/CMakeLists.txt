# rtd —— Jetson 实时控制守护进程
# C++17, 依赖: libzmq (cppzmq 头文件 /usr/include/zmq.hpp) + nlohmann/json (header-only)
cmake_minimum_required(VERSION 3.10)
project(rtd CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")

# 优先用 pkg-config 找 libzmq; 找不到则回退到 -lzmq
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBZMQ QUIET IMPORTED_TARGET libzmq)
endif()
if(LIBZMQ_FOUND)
  message(STATUS "libzmq via pkg-config: ${LIBZMQ_VERSION}")
  set(ZMQ_TARGET PkgConfig::LIBZMQ)
else()
  message(STATUS "libzmq via -lzmq fallback")
  find_library(ZMQ_LIB NAMES zmq REQUIRED)
  set(ZMQ_TARGET ZMQ_LIB)
endif()

add_executable(rtd
  src/main.cpp
  src/util.cpp
  src/protocol.cpp
  src/kinematics.cpp
  src/serial.cpp
  src/zmq_api.cpp
  src/rtd.cpp
)
target_include_directories(rtd PRIVATE src)
target_link_libraries(rtd PRIVATE ${ZMQ_TARGET} pthread)
