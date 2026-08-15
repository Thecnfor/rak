#!/usr/bin/env bash
# 构建 TensorRT FP16 引擎(lane + task), 摆脱 paddle/onnxruntime 运行时依赖。
#
# 前置: 已安装 paddle2onnx(仅用于 pdmodel->onnx 转换, 不依赖 paddle 本体)
#       + TensorRT(本机 /usr/src/tensorrt/bin/trtexec, TRT 10.3 / CUDA 12.6)
#
# 建议: 重启后的干净环境(未开 IDE)下执行, 避免 3.5GB 内存被占导致 NvMap 分配失败。
# 用法:  bash build_trt_engines.sh [--rebuild]
set -uo pipefail
cd "$(dirname "$0")"

ENGINE_DIR="$PWD/trt_engines"
MODELS_DIR="smartcar/paddlebaidu/models"
TRTEXEC=/usr/src/tensorrt/bin/trtexec
REBUILD=0
[ "${1:-}" = "--rebuild" ] && REBUILD=1

mkdir -p "$ENGINE_DIR"

# 释放页缓存, 尽量给连续内存让路 (无 sudo 时忽略)
if sudo -n true 2>/dev/null; then
    sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches' 2>/dev/null || true
fi

build_one() {
    local name=$1 model_dir=$2 model_file=$3 params_file=$4
    local fix_onx=$5 shapes=$6
    local onnx="$ENGINE_DIR/$name.onnx"
    local engine="$ENGINE_DIR/${name}_fp16.engine"

    if [ -f "$engine" ] && [ $REBUILD -eq 0 ]; then
        echo "[skip] $engine 已存在"
        return
    fi

    if [ ! -f "$onnx" ]; then
        echo "==> [$name] pdmodel -> onnx"
        paddle2onnx --model_dir "$MODELS_DIR/$model_dir" \
            --model_filename "$model_file" --params_filename "$params_file" \
            --save_file "$onnx" --opset_version 13 >/dev/null 2>&1
    fi

    # 部分检测模型需要静态化输入 + 补 Squeeze.axes 才能被 TRT 解析
    local src_onnx="$onnx"
    if [ -n "$fix_onx" ]; then
        python3 "$(dirname "$0")/fix_onnx_for_trt.py" "$onnx" "$ENGINE_DIR/${name}_trt.onnx"
        src_onnx="$ENGINE_DIR/${name}_trt.onnx"
    fi

    echo "==> [$name] 构建 fp16 引擎"
    local args=""
    [ -n "$shapes" ] && args="--shapes=$shapes"
    timeout 900 $TRTEXEC --onnx="$src_onnx" --saveEngine="$engine" --fp16 \
        --memPoolSize=workspace:512 $args 2>&1 \
        | grep -E "PASSED|FAILED|Engine created|out of memory|error 12" | head -5
    echo "    -> $(ls -la "$engine" 2>/dev/null | awk '{print $5}') bytes"
}

# lane: CLRNet, 输入固定 1x3x128x128
build_one lane lane_model cnn_lane.pdmodel cnn_lane.pdiparams "" "inputs:1x3x128x128"
# correction: 居中/回正 CNN, 单 steer 输出, 输入固定 1x3x128x128(输入名 inputs)
build_one correction correction_model correction.pdmodel correction.pdiparams "" "inputs:1x3x128x128"
# task: PP-YOLOE 检测, 静态化输入 + 补 Squeeze axes 后构建
build_one task task2026 model.pdmodel model.pdiparams fix "image:1x3x640x640,scale_factor:1x2"

echo "=== 完成, 引擎文件: ==="
ls -la "$ENGINE_DIR"/*.engine 2>/dev/null || echo "未生成任何引擎"