#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""correction_cnn 导出脚本: .pdparams -> 静态图(.pdmodel+.pdiparams) -> ONNX。

correction_cnn 训练产物是 paddle 动态图权重 (.pdparams), 而新仓库(6_lost_car)
的推理链是 TensorRT(trt_fp16), 运行时绝不加载 paddle。本脚本是**一次性开发工具**,
在开发机/装好 paddle 的机器上把权重转成 TRT 链的原料:
  .pdparams -> 静态图(输入名固定为 inputs, 与 TrtLaneInfer 对齐) -> ONNX(opset 13,
  与 build_trt_engines.sh 的 lane/task 一致)。

之后由 scripts/build_trt_engines.sh 里的 trtexec 把 onnx 建成 .engine(需车上 GPU)。

用法(在仓库根目录):
  python scripts/export_correction_onnx.py \
      --weights /path/to/correction_cnn.pdparams \
      [--out smartcar/paddlebaidu/models/correction_model] [--opset 13]

依赖(仅开发期): paddle、paddle2onnx、onnx、onnxruntime。
runtime 不依赖本脚本。
"""
import argparse
import os
import sys

import numpy as np
import paddle
import paddle2onnx
from paddle.static import InputSpec


class CorrectionCNN(paddle.nn.Layer):
    """与 correction_cnn_v1/smartcar/.../base/correction_cnn.py 同步。
    输入 128x128x3 RGB, 输出 steer ∈ [-1, 1]。
    """

    def __init__(self):
        super().__init__()
        self.features = paddle.nn.Sequential(
            paddle.nn.Conv2D(3, 16, 3, stride=2, padding=1), paddle.nn.ReLU(),    # 128->64
            paddle.nn.Conv2D(16, 32, 3, stride=2, padding=1), paddle.nn.ReLU(),   # 64->32
            paddle.nn.Conv2D(32, 64, 3, stride=2, padding=1), paddle.nn.ReLU(),   # 32->16
            paddle.nn.Conv2D(64, 64, 3, stride=2, padding=1), paddle.nn.ReLU(),   # 16->8
            paddle.nn.Conv2D(64, 64, 3, stride=2, padding=1), paddle.nn.ReLU(),   # 8->4
            paddle.nn.Conv2D(64, 64, 3, stride=2, padding=1), paddle.nn.ReLU(),   # 4->2
        )
        self.head = paddle.nn.Sequential(
            paddle.nn.Linear(64 * 2 * 2, 64),
            paddle.nn.ReLU(),
            paddle.nn.Linear(64, 32),
            paddle.nn.ReLU(),
            paddle.nn.Linear(32, 1),
            paddle.nn.Tanh(),
        )

    def forward(self, x):
        x = self.features(x)
        x = paddle.flatten(x, 1)
        return self.head(x)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--weights", required=True,
                    help="correction_cnn.pdparams 权重路径")
    ap.add_argument("--out", default="smartcar/paddlebaidu/models/correction_model",
                    help="输出目录(静态图 + onnx), 默认仓库内 models/correction_model")
    ap.add_argument("--opset", type=int, default=13,
                    help="ONNX opset 版本, 默认 13(与 build_trt_engines.sh 一致)")
    args = ap.parse_args()

    if not os.path.exists(args.weights):
        print(f"[FAIL] 权重不存在: {args.weights}")
        return 1
    os.makedirs(args.out, exist_ok=True)

    # 1) 动态图加载权重
    model = CorrectionCNN()
    model.set_state_dict(paddle.load(args.weights))
    model.eval()
    print(f"[OK] 动态图权重加载完成: {args.weights}")

    # 2) .pdparams -> 静态图(输入名固定为 inputs, 与 TrtLaneInfer 对齐)
    static_model = paddle.jit.to_static(
        model,
        input_spec=[InputSpec([None, 3, 128, 128], "float32", "inputs")],
    )
    static_prefix = os.path.join(args.out, "correction")
    paddle.jit.save(static_model, static_prefix)
    for suffix in (".pdmodel", ".pdiparams", ".pdiparams.info"):
        p = static_prefix + suffix
        print(f"   {suffix}: {p} ({os.path.getsize(p)} B)" if os.path.exists(p)
              else f"   {suffix}: 缺失!")
    if not os.path.exists(static_prefix + ".pdiparams"):
        print("[FAIL] 静态图缺少 .pdiparams, 请检查 paddle 版本")
        return 1
    print("[OK] 静态图导出完成")

    # 3) 静态图 -> ONNX(opset 与 build_trt_engines.sh 一致)
    onnx_path = os.path.join(args.out, "correction.onnx")
    paddle2onnx.export(
        model_filename=static_prefix + ".pdmodel",
        params_filename=static_prefix + ".pdiparams",
        save_file=onnx_path,
        opset_version=args.opset,
    )
    print(f"[OK] ONNX 导出完成: {onnx_path} ({os.path.getsize(onnx_path)} B)")

    # 4) 数值自检: 动态图 vs 静态图 vs ONNX(同一随机输入)
    import onnx
    import onnxruntime as ort

    m = onnx.load(onnx_path)
    in_name = m.graph.input[0].name
    out_name = m.graph.output[0].name
    print(f"   ONNX 输入: {in_name}  输出: {out_name}")

    x = (np.random.rand(1, 3, 128, 128).astype(np.float32) - 0.5) * 2

    with paddle.no_grad():
        dyn = float(model(paddle.to_tensor(x)).numpy()[0, 0])

    from paddle.inference import Config, create_predictor
    cfg = Config(static_prefix + ".pdmodel", static_prefix + ".pdiparams")
    cfg.enable_use_gpu(100, 0)
    cfg.switch_ir_optim()
    predictor = create_predictor(cfg)
    ih = predictor.get_input_handle(predictor.get_input_names()[0])
    ih.copy_from_cpu(x)
    predictor.run()
    oh = predictor.get_output_handle(predictor.get_output_names()[0])
    stat = float(np.asarray(oh.copy_to_cpu()).ravel()[0])

    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    o = float(np.asarray(sess.run(None, {in_name: x})[0]).ravel()[0])

    print(f"[OK] 数值自检: dyn={dyn:+.6f} static={stat:+.6f} onnx={o:+.6f}")
    max_diff = max(abs(stat - dyn), abs(o - dyn))
    if max_diff < 1e-4:
        print("[OK] 三者一致, 可进入 build_trt_engines.sh 构建")
        return 0
    print(f"[FAIL] 三者不一致, max_diff={max_diff}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
