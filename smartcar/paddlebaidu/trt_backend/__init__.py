# -*- coding: utf-8 -*-
"""TensorRT 推理后端(paddle 免费版)。

只依赖 tensorrt + libcudart(ctypes 封装)+ numpy/cv2。
提供与 paddle 路径接口一致的 TrtLaneInfer / TrtYoloeInfer / TrtCorrectionInfer。
"""
from .trt_infer import TrtLaneInfer, TrtYoloeInfer, TrtCorrectionInfer

__all__ = ["TrtLaneInfer", "TrtYoloeInfer", "TrtCorrectionInfer"]