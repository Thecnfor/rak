# -*- coding: utf-8 -*-
"""TensorRT 推理运行时: 替代 paddle_inference 的 LaneInfer / YoloeInfer。

设计目标: 运行时完全不依赖 paddle, 只依赖 tensorrt + cuda(ctypes 封装)+
numpy/cv2。预处理/后处理逻辑与 __paddle_jetson_ 保持一致, 保证输出格式
(列表 shape [cls_id, obj_id, label, score, ...]) 与 paddle 路径完全兼容。

引擎文件位置: <repo_root>/trt_engines/<arch>_fp16.engine
  lane -> lane_fp16.engine   输入 image: None
  task -> task_fp16.engine   输入 image + scale_factor (YOLO 检测)
"""

import os
import threading

import cv2
import numpy as np
import tensorrt as trt

from . import cuda_utils
from .preprocess import (
    LetterBoxResize,
    NormalizeImage,
    Pad,
    PadStride,
    Permute,
    Resize,
    ShortSizeScale,
    WarpAffine,
    preprocess,
)
from .utils import nms

# repo_root = smartcar/paddlebaidu/trt_backend -> 上 3 层到项目根
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_MODELS_DIR = os.path.join(_REPO_ROOT, "smartcar", "paddlebaidu", "models")
TRT_ENGINE_DIR = os.environ.get(
    "TRT_ENGINE_DIR", os.path.join(_REPO_ROOT, "trt_engines")
)

# model_dir(config) -> 引擎文件名
ENGINE_FILE = {
    "lane_model": "lane_fp16.engine",
    "task2026": "task_fp16.engine",
    "correction_model": "correction_fp16.engine",
}

# 动态输出维度的预算上限(rows), 超出会报错 — 实际检测框数量远小于此
_DYN_BUDGET = 4096


def _load_yaml(path):
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class DetectResult:
    """检测结果, 与 paddle 路径的 infer_wrap.DetectResult 行为一致。"""

    def __init__(self, category_id, label, score, bbox, object_id=0):
        self.class_id = int(category_id)
        self.object_id = int(object_id)
        self.label_name = label
        self.score = float(score)
        self.bbox = bbox.astype(np.int32)
        if self.bbox[0] < 0:
            self.bbox[0] = 0
        if self.bbox[1] < 0:
            self.bbox[1] = 0
        if self.bbox[2] > 639:
            self.bbox[2] = 639
        if self.bbox[3] > 479:
            self.bbox[3] = 479
        self.center = [self.bbox[0] + self.bbox[2] / 2, self.bbox[1] + self.bbox[3] / 2]
        self.middle = [320, 240]

    def tolist_nomoralize(self, size):
        mid_x = size[0] / 2
        mid_y = size[1] / 2
        pt_mid = [mid_x, mid_y]
        normalized_x = float((self.bbox[0] + self.bbox[2]) / 2 - pt_mid[0]) / pt_mid[0]
        normalized_y = float((self.bbox[1] + self.bbox[3]) / 2 - pt_mid[1]) / pt_mid[1]
        normalized_w = float(self.bbox[2] - self.bbox[0]) / pt_mid[0]
        normalized_h = float(self.bbox[3] - self.bbox[1]) / pt_mid[1]
        return [self.class_id, self.object_id, self.label_name, self.score] + [
            normalized_x,
            normalized_y,
            normalized_w,
            normalized_h,
        ]

    def __repr__(self):
        return "cls_id:{} obj_id:{} label:{} score:{:.3f} bbox:{}".format(
            self.class_id,
            self.object_id,
            self.label_name,
            self.score,
            self.bbox.tolist(),
        )


class TrtEngine:
    """串行 .engine 引擎封装: H2D -> run -> D2H。"""

    def __init__(self, engine_path):
        self._logger = trt.Logger(trt.Logger.ERROR)
        runtime = trt.Runtime(self._logger)
        with open(engine_path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # 收集输入/输出张量信息并分配设备内存
        self._tensors = []  # (name, is_input, shape_with_dyn, np_dtype)
        self._ptrs = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            shape = tuple(self.engine.get_tensor_shape(name))
            np_dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            # 动态维(-1)按预算分配
            shape_fixed = tuple(_DYN_BUDGET if d < 0 else d for d in shape)
            nbytes = int(np.prod(shape_fixed)) * np_dtype.itemsize
            self._ptrs[name] = cuda_utils.mem_alloc(nbytes)
            self._tensors.append((name, is_input, shape, np_dtype))
        self._outputs = [t for t in self._tensors if not t[1]]

    @property
    def output_names(self):
        return [t[0] for t in self._outputs]

    def __del__(self):
        ptrs = getattr(self, "_ptrs", {})
        for ptr in ptrs.values():
            try:
                cuda_utils.mem_free(ptr)
            except Exception:
                pass

    def infer(self, inputs):
        """inputs: {tensor_name: np.ndarray}; 返回 {tensor_name: np.ndarray}。"""
        stream = cuda_utils.stream(threading.get_ident())

        for name, arr in inputs.items():
            arr = np.ascontiguousarray(arr)
            if arr.dtype != self._input_np_dtype(name):
                # 引擎输入一般是 float32
                arr = arr.astype(self._input_np_dtype(name))
            cuda_utils.memcpy_htod(self._ptrs[name], arr)
            self.context.set_tensor_address(name, self._ptrs[name])

        for name, is_input, shape, np_dtype in self._outputs:
            self.context.set_tensor_address(name, self._ptrs[name])

        self.context.execute_async_v3(stream)
        cuda_utils.stream_sync(stream)

        results = {}
        for name, is_input, shape, np_dtype in self._outputs:
            actual = tuple(self.context.get_tensor_shape(name))
            n = int(np.prod([abs(d) for d in actual]))
            buf = np.empty(n, dtype=np_dtype)
            cuda_utils.memcpy_dtoh(buf, self._ptrs[name], n * np_dtype.itemsize)
            results[name] = buf.reshape(actual)
        return results

    def _input_np_dtype(self, name):
        for n, is_input, shape, np_dtype in self._tensors:
            if n == name and is_input:
                return np_dtype
        raise KeyError(name)


class TrtYoloeInfer:
    """YOLO 检测 (替代 YoloeInfer)。输出格式与 paddle 路径兼容。"""

    def __init__(self, model_dir="task2026", run_mode="trt_fp16"):
        self.model_dir = model_dir
        self.run_mode = run_mode
        engine_path = os.path.join(TRT_ENGINE_DIR, ENGINE_FILE[model_dir])
        self.engine = TrtEngine(engine_path)

        # 读取模型预处理配置与标签
        cfg = _load_yaml(os.path.join(_MODELS_DIR, model_dir, "infer_cfg.yml"))
        preprocess_ops = []
        for op_info in cfg["Preprocess"]:
            op = op_info.copy()
            op_type = op.pop("type")
            preprocess_ops.append(eval(op_type)(**op))
        self.ops = preprocess_ops
        self.threshold = cfg.get("draw_threshold", 0.5)
        self.label_list = cfg["label_list"]

    def _run(self, image_rgb):
        im, im_info = preprocess(image_rgb, self.ops)  # im = CHW float32
        inputs = {
            "image": np.ascontiguousarray(im)[np.newaxis, :],
            "scale_factor": np.ascontiguousarray(im_info["scale_factor"])[
                np.newaxis, :
            ],
        }
        out = self.engine.infer(inputs)
        # 输出顺序: 按 engine 输出索引 -> boxes, boxes_num
        names = self.engine.output_names
        return out.get(names[0]), out.get(names[1])

    def predict(self, image, normalize_out=False):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        boxes, boxes_num = self._run(image_rgb)
        # 与 paddle Detector.filter_box 一致
        np_boxes_num = boxes_num
        boxes_all = boxes
        start_idx = 0
        filter_boxes = []
        for i in range(len(np_boxes_num)):
            bnum = int(np_boxes_num[i])
            boxes_i = boxes_all[start_idx : start_idx + bnum, :]
            idx = boxes_i[:, 1] > self.threshold
            filter_boxes.append(boxes_i[idx, :])
            start_idx += bnum
        det_res = {"boxes": np.concatenate(filter_boxes)}

        # 与 paddle YoloeInfer.predict 一致(含 nms 调用方式)
        det = nms(det_res["boxes"], len(self.label_list))
        ret = []
        for bbox in det:
            cls_id, score, rect = int(bbox[0]), bbox[1], bbox[2:].astype(np.int32)
            res = DetectResult(cls_id, self.label_list[cls_id], score, rect)
            if normalize_out:
                res = res.tolist_nomoralize(image.shape[:2][::-1])
            ret.append(res)
        return ret

    def __call__(self, image, *args, **kwds):
        return self.predict(image, *args, **kwds)

    def close(self):
        del self.engine


class TrtLaneInfer:
    """车道线 (替代 LaneInfer)。复刻 paddle 版预处理与输出语义。"""

    def __init__(self, model_dir="lane_model", run_mode="trt_fp16"):
        self.model_dir = model_dir
        self.run_mode = run_mode
        engine_path = os.path.join(TRT_ENGINE_DIR, ENGINE_FILE[model_dir])
        self.engine = TrtEngine(engine_path)
        self.img_size = (128, 128)
        self.mean = 1.0

    def _preprocess(self, img):
        img = cv2.resize(img, self.img_size)
        img = img.astype(np.float32) / 127.5 - self.mean  # 与 paddle 版一致
        img = img[:, :, ::-1].astype("float32")  # bgr -> rgb
        img = img.transpose((2, 0, 1))  # hwc -> chw
        return np.ascontiguousarray(img)[np.newaxis, :]

    def predict(self, image, normalize_out=False):
        x = self._preprocess(image)
        out = self.engine.infer({"inputs": x})
        name = next(iter(out))
        output_data = out[name][0]
        if normalize_out:
            return output_data.tolist()
        return output_data

    def __call__(self, image, *args, **kwds):
        return self.predict(image, *args, **kwds)

    def close(self):
        del self.engine


class TrtCorrectionInfer:
    """correction CNN (替代动态图 CorrectionInfer)。单 steer 输出。

    输入 128x128 cam1 帧, 输出 steer ∈ [-1, +1]。
    预处理与 TrtLaneInfer 完全一致: 两模型都以 RGB 训练, 摄像头帧是 BGR,
    所以 _preprocess 同样做 /127.5-1 + BGR->RGB + CHW。输出是单个标量
    (shape (1,)), predict 返回 [steer]。
    """

    def __init__(self, model_dir="correction_model", run_mode="trt_fp16"):
        self.model_dir = model_dir
        self.run_mode = run_mode
        engine_path = os.path.join(TRT_ENGINE_DIR, ENGINE_FILE[model_dir])
        self.engine = TrtEngine(engine_path)
        self.img_size = (128, 128)
        self.mean = 1.0

    def _preprocess(self, img):
        img = cv2.resize(img, self.img_size)
        img = img.astype(np.float32) / 127.5 - self.mean  # 与 TrtLaneInfer 一致
        img = img[:, :, ::-1].astype("float32")  # bgr -> rgb
        img = img.transpose((2, 0, 1))  # hwc -> chw
        return np.ascontiguousarray(img)[np.newaxis, :]

    def predict(self, image, normalize_out=False):
        x = self._preprocess(image)
        out = self.engine.infer({"inputs": x})
        name = next(iter(out))
        output_data = out[name][0]  # shape (1,) -> [steer]
        if normalize_out:
            return output_data.tolist()
        return output_data

    def __call__(self, image, *args, **kwds):
        return self.predict(image, *args, **kwds)

    def close(self):
        del self.engine
