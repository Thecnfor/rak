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


def fix_squeeze7(graph):
    """把 p2o.Squeeze.7 换成 Reshape([-1, 6]), 修复多目标输出被截成 1 框的 bug。

    paddle2onnx 导出的 NMS 输出链: Concat.39 = [1, N, 6](batch=1, N 框, 6 字段)
    -> Squeeze.7(axes=[0]) -> [N, 6]。ONNX 逻辑正确, 但该 Squeeze 的 axes 是
    动态常量输入, TRT 编译动态 shape 时把维序搞错, 实际压掉了框数维, 引擎输出
    固定为 [1, 6](永远只有 1 个框; boxes_num 却正确报 N)。用 Reshape([-1, 6])
    替换, 无论 TRT 把动态维解释成 [1, N, 6] 还是 [N, 1, 6], 都正确展成 [N, 6]。
    """
    for node in graph.nodes:
        if node.op == "Squeeze" and node.name == "p2o.Squeeze.7":
            out_var = node.outputs[0]  # 复用已有输出变量(它同时是 graph output)
            shape_const = gs.Constant(
                name="p2o.Squeeze.7/reshape_shape",
                values=np.array([-1, 6], dtype=np.int64),
            )
            graph.nodes.append(
                gs.Node(
                    op="Reshape",
                    name="p2o.Reshape.7_fix",
                    inputs=[node.inputs[0], shape_const],
                    outputs=[out_var],
                )
            )
            for o in node.outputs:
                o.inputs = [i for i in o.inputs if i.name != node.name]
            node.outputs = []
            return True
    return False


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

    if fix_squeeze7(graph):
        changed.append("p2o.Squeeze.7 -> Reshape([-1, 6])")

    graph.cleanup().toposort()
    onnx.save(gs.export_onnx(graph), dst)
    print(f"已修复 {changed} -> {dst}")


if __name__ == "__main__":
    main()