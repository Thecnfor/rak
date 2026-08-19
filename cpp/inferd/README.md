# inferd — C++ TensorRT 推理守护进程（lane:5001 / correction:5003）

用 C++ 接管 Python 推理后端中 GPU 推理 <1ms、被 Python 外围开销（JPEG 解码、
numpy 预处理、TRT binding、JSON）拖慢的两个小模型端口，`task:5002` 仍由原
Python 后端（`smartcar/paddlebaidu/infer_cs/base/infer_back_end.py`）接管。

## 功能

- 单进程双线程：lane(5001) / correction(5003) 各一个后台线程 + 一个独立
  TRT 引擎实例（与 Python 端「每端口一线程 + 独立引擎」模型一致）。
- 引擎直接 `deserializeCudaEngine` 加载已存在的 `trt_engines/lane_fp16.engine`
  与 `correction_fp16.engine`（TRT 10.3，无需重新构建）。
- 预处理逐语义复刻 `TrtLaneInfer._preprocess` / `TrtCorrectionInfer._preprocess`：
  双线性 resize 128x128 → `/127.5 - 1.0` → BGR→RGB → HWC→CHW。
- 全 C++ 实现，无 OpenCV / 无 nvcc（只用 `cuda_runtime.h` 主机 API）。

## 协议（与 Python 后端兼容，见 `infer_back_end.py::process_demo`）

REQ/REP，`bind tcp://127.0.0.1:<port>`，单帧请求：

| 请求头 | 请求体 | 回复 |
|---|---|---|
| `b"ATATA"` | — | JSON 布尔：引擎就绪 `true` / 加载中或加载失败 `false` |
| `b"rawi"` | 小端 `uint32 h` + 小端 `uint32 w` + `h*w*3` 字节 BGR 连续像素 | JSON float 数组（第一个输出张量全部元素，等价 `output_data.tolist()`）；形状不符或未就绪回 `[]` |
| `b"image"` | JPEG（旧协议） | 不支持，恒回 `[]` |
| 其他 | — | `[]` |

注意：回复是 **单个 ZMQ 消息帧**，客户端 `json.loads` 直接解析；字面量
`true`/`false` 不能带引号。`ATATA` 在端口绑定后、引擎加载完成前返回
`false`，`rawi` 返回 `[]`（与 Python `flag_infer_initok` 语义一致）。

## 依赖（Jetson aarch64 实测）

- TensorRT 10.3：头 `/usr/include/aarch64-linux-gnu/NvInferRuntime.h`，库
  `/usr/lib/aarch64-linux-gnu/libnvinfer.so`
- CUDA 12.6：头 `/usr/local/cuda-12.6/targets/aarch64-linux/include`，库
  `/usr/local/cuda-12.6/targets/aarch64-linux/lib/libcudart.so`
- cppzmq：`/usr/include/zmq.hpp`（libzmq 4.3.4）
- nlohmann/json：`/usr/include/nlohmann/json.hpp`
- g++ 需支持 C++17

## 构建

```bash
mkdir -p /home/xrak/rak/cpp/inferd/build
cd /home/xrak/rak/cpp/inferd/build
cmake .. && make
# 产物: build/inferd
```

只编译、不运行。**注意：本机 5001/5002/5003 常被车上常驻 Python 后端占用，
GPU 内存紧张，构建验证时切勿直接运行 inferd，也不建议跑 --selftest**
（--selftest 只加载引擎不 bind 端口，但会临时占用 GPU 显存）。

## 部署

1. 构建出 `build/inferd`。
2. 复制并启用 systemd 服务：

```bash
sudo cp /home/xrak/rak/scripts/infer-cpp.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now infer-cpp.service
journalctl -u infer-cpp.service -f   # 查看日志
```

3. 自检（可选，需先停掉占用端口的服务，见下）：

```bash
./build/inferd --selftest
```

## 切换与回退

### 切换（Python 后端 → C++ inferd 接管 lane/correction）

Python 后端一次 bind 三个端口；要让出 5001/5003，必须把它的 `infer_cfg`
裁到只剩 task，否则两端抢端口：

1. 编辑 `config_car.yml` 的 `infer_cfg`：删除 `lane`、`correction` 两条，
   仅保留 `task`（5002）。
2. 重启 Python 后端：`sudo systemctl restart infer-backend.service`
   （确认 `journalctl -u infer-backend` 无 5001/5003 bind 报错）。
3. 启用 inferd：`sudo systemctl enable --now infer-cpp.service`。
4. 冒烟：向 5001/5003 发 `ATATA` 应回 `true`；发一条 `rawi` 帧应回数组。

### 回退（inferd → Python 后端）

1. `sudo systemctl disable --now infer-cpp.service`
2. 还原 `config_car.yml` 的 `infer_cfg`（补回 `lane`、`correction`）。
3. `sudo systemctl restart infer-backend.service`

## 遗留风险 / 注意事项

- **输出仅序列化第一个张量**：与 Python `next(iter(out))` 一致；若未来引擎
  输出多于一个且客户端依赖更多输出，需要扩展 `TrtModel`。
- **输出形状要求静态**：lane/correction 引擎输出为固定 shape；若改为动态
  输出（-1 维），`load()` 会直接报错，需另行处理。
- **设备内存用 `cudaMalloc` 而非 `cudaMallocAsync`**：`cudaMallocAsync` 要求
  sm_70+，Jetson Nano（Maxwell, sm_53）不支持，兼容性优先。
- **字节序**：`rawi` 帧头按小端解析，直接 `memcpy` 到 `uint32_t`（
  aarch64/x86 均为小端）。
- **JPEG（`image`）协议不支持**：C++ 侧未引入解码库，恒回 `[]`；当前车端
  客户端已全部改用 `rawi`。
- **双模型并行加载**：两个线程各自 load，短暂并行占用 GPU 显存（两引擎合计
  <2MB），在 Python 后端常驻的前提下仍在预算内。
- 引擎路径/端口均可通过命令行覆盖，systemd 服务未传参即用默认值。
