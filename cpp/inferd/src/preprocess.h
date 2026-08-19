// preprocess.h
//
// 输入预处理, 逐语义复刻 Python 端 TrtLaneInfer._preprocess /
// TrtCorrectionInfer._preprocess:
//   img = cv2.resize(img, (128, 128))            # 双线性
//   img = img.astype(np.float32) / 127.5 - 1.0   # 归一化
//   img = img[:, :, ::-1]                        # BGR -> RGB
//   img = img.transpose((2, 0, 1))               # HWC -> CHW
//
// 合并后的等价公式(先 resize 后翻转):
//   out[c][y][x] = resize_bgr[y][x][2-c] / 127.5 - 1.0
// 其中 resize_bgr 是双线性缩放后的 uint8 BGR 图(与 cv2.resize 输出同语义)。
#pragma once

#include <cstddef>
#include <cstdint>

namespace inferd {

// 模型固定输入尺寸(lane / correction 都是 1x3x128x128)
constexpr int kInputH = 128;
constexpr int kInputW = 128;
constexpr int kInputC = 3;

// 输入 CHW fp32 缓冲的 float 个数
constexpr size_t kInputFloats =
    static_cast<size_t>(kInputH) * kInputW * kInputC;

// 将 BGR 连续帧(src_bgr, 行优先 h*w*3 字节)预处理到 out_chw(CHW 连续 fp32)。
// 双线性采样按 cv2 INTER_LINEAR 的中心对齐映射, 边界越界 clamp 到边缘像素。
// 返回 false 表示 src_h/src_w 非法(<=0); 调用方应先按 h*w*3 校验帧体长度。
bool preprocess_bgr(const uint8_t* src_bgr, int src_h, int src_w,
                    float* out_chw);

}  // namespace inferd
