# -*- coding: utf-8 -*-
"""检测结果归一化坐标 → 像素坐标的统一换算(唯一事实来源)。

task 模型输出格式 [cls_id, obj_id, label, score, x_c, y_c, w, h]:
  - x_c, y_c: 中心归一化坐标 ∈ [-1, 1], 0=图像中心, ±1=图像边缘
  - w, h: 半宽/半高归一 ∈ [0, 2], 1.0 = 半幅画面宽/高
产出见 smartcar/paddlebaidu/trt_backend/trt_infer.py DetectResult.tolist_nomoralize。
所有把归一化 bbox 画到图像/裁剪的调用方都必须走这里, 不要各自内联换算。
"""


def norm_center_to_pixel(x_c, y_c, img_w, img_h):
    """归一化中心坐标 → 像素中心 (x_px, y_px)。"""
    return (x_c + 1) / 2 * img_w, (y_c + 1) / 2 * img_h


def norm_box_to_pixel(x_c, y_c, w, h, img_w, img_h, scale=1.0):
    """归一化 bbox → 像素角点 (x1, y1, x2, y2)。

    scale > 1 时按比例放大框(裁剪留边用), 用法与旧 ocr._bbox_to_pixel 一致。
    """
    xc = int((x_c + 1) / 2 * img_w)
    yc = int((y_c + 1) / 2 * img_h)
    bw = int(w * scale * img_w / 2)
    bh = int(h * scale * img_h / 2)
    return int(xc - bw / 2), int(yc - bh / 2), int(xc + bw / 2), int(yc + bh / 2)
