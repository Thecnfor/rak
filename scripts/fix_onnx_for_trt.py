#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 paddle2onnx 导出的检测模型 ONNX 静态化, 供 TensorRT 导入。

paddle2onnx 导出的 YOLO 检测模型(含 NMS)输入是动态 batch:
  image:       [? ,3,640,640]
  scale_factor: [? ,2]
TensorRT 对动态 shape 下的 Squeeze 算子无法推导 axes("Cannot infer squeeze
dimensions from a dynamic shape"), 因此把两个输入的维度固定为静态。

用法:
  python fix_onnx_for_trt.py trt_engines/task.onnx trt_engines/task_trt.onnx
"""
import sys

import numpy as np
import onnx
import onnx_graphsurgeon as gs

# 输入名 -> 固定形状 (batch=1, 与运行时代码一致)
DEFAULT_SHAPES = {
    "image": [1, 3, 640, 640],
    "scale_factor": [1, 2],
}

# 需要显式补 axes 的 Squeeze 节点: 这些节点由 paddle2onnx 的 NMS 翻译产生,
# axes 缺失且输入 shape 为动态, TensorRT 无法推导。
# 目标形状 [dyn, 1] -> 压掉最后的 singleton 维 (即 axis=1)。
SQUEEZE_AXES = {
    "p2o.Squeeze.3": [1],
    "p2o.Squeeze.5": [1],
}


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "trt_engines/task.onnx"
    dst = sys.argv[2] if len(sys.argv) > 2 else "trt_engines/task_trt.onnx"

    graph = gs.import_onnx(onnx.load(src))
    changed = []
    for inp in graph.inputs:
        if inp.name in DEFAULT_SHAPES:
            inp.shape = DEFAULT_SHAPES[inp.name]
            changed.append(inp.name)

    for node in graph.nodes:
        if node.op == "Squeeze" and node.name in SQUEEZE_AXES:
            axes = np.array(SQUEEZE_AXES[node.name], dtype=np.int64)
            node.inputs.append(
                gs.Constant(name=f"{node.name}/axes", values=axes))
            changed.append(node.name)

    graph.cleanup().toposort()
    onnx.save(gs.export_onnx(graph), dst)
    print(f"已修复 {changed} -> {dst}")


if __name__ == "__main__":
    main()