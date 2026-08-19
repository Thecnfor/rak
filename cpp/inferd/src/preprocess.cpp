// preprocess.cpp
#include "preprocess.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace inferd {

namespace {

// 对单个通道做双线性插值(x0<=x1, y0<=y1, 已 clamp)。
inline float bilinear_channel(const uint8_t* bgr, int sw, int x0, int y0,
                              int x1, int y1, float fx, float fy, int c) {
    const float v00 = static_cast<float>(bgr[(y0 * sw + x0) * 3 + c]);
    const float v01 = static_cast<float>(bgr[(y0 * sw + x1) * 3 + c]);
    const float v10 = static_cast<float>(bgr[(y1 * sw + x0) * 3 + c]);
    const float v11 = static_cast<float>(bgr[(y1 * sw + x1) * 3 + c]);
    // 4 点加权: 权重与 cv2 INTER_LINEAR 完全一致
    return v00 * (1.0f - fx) * (1.0f - fy) + v01 * fx * (1.0f - fy) +
           v10 * (1.0f - fx) * fy + v11 * fx * fy;
}

}  // namespace

bool preprocess_bgr(const uint8_t* src_bgr, int src_h, int src_w,
                    float* out_chw) {
    if (src_h <= 0 || src_w <= 0 || src_bgr == nullptr || out_chw == nullptr) {
        return false;
    }

    // 双线性缩放比例(目标固定 128x128)
    const float scale_x = static_cast<float>(src_w) / kInputW;
    const float scale_y = static_cast<float>(src_h) / kInputH;
    const int xmax = src_w - 1;
    const int ymax = src_h - 1;

    // 第一步: 双线性 resize 到 128x128, 结果仍是 uint8 BGR。
    // 用中间 uint8 缓冲复刻 cv2.resize 的语义(先缩放成 8bit, 再归一化),
    // 避免直接浮点插值引入的舍入差异。128*128*3 = 49KB, 栈/堆均可。
    std::vector<uint8_t> resized(static_cast<size_t>(kInputH) * kInputW * kInputC);

    for (int y = 0; y < kInputH; ++y) {
        // cv2 INTER_LINEAR 的中心对齐映射: dst 中心反投影到 src
        // 坐标, 再 clamp 到 [0, src-1]。缩放(目标>源)时 fx<0 属正常,
        // clamp 后等效于最近邻边缘像素, 与 cv2 一致。
        float fy = (y + 0.5f) * scale_y - 0.5f;
        int y0 = static_cast<int>(std::floor(fy));
        fy -= static_cast<float>(y0);
        if (y0 < 0) {
            y0 = 0;
            fy = 0.0f;
        } else if (y0 >= ymax) {
            y0 = ymax;
            fy = 0.0f;
        }
        const int y1 = std::min(y0 + 1, ymax);  // 边界 clamp, 权重已归零

        for (int x = 0; x < kInputW; ++x) {
            float fx = (x + 0.5f) * scale_x - 0.5f;
            int x0 = static_cast<int>(std::floor(fx));
            fx -= static_cast<float>(x0);
            if (x0 < 0) {
                x0 = 0;
                fx = 0.0f;
            } else if (x0 >= xmax) {
                x0 = xmax;
                fx = 0.0f;
            }
            const int x1 = std::min(x0 + 1, xmax);

            for (int c = 0; c < 3; ++c) {
                const float v =
                    bilinear_channel(src_bgr, src_w, x0, y0, x1, y1, fx, fy, c);
                // +0.5 四舍五入到 uint8, 对齐 cv2.resize 对 8bit 输出的取整
                resized[(static_cast<size_t>(y) * kInputW + x) * 3 + c] =
                    static_cast<uint8_t>(std::min(255.0f, v + 0.5f));
            }
        }
    }

    // 第二步: BGR->RGB + HWC->CHW + /127.5-1.0 一次完成。
    // 注意翻转在归一化之后再交换通道: out[c][y][x] = bgr[y][x][2-c]。
    const float inv = 1.0f / 127.5f;
    for (int y = 0; y < kInputH; ++y) {
        for (int x = 0; x < kInputW; ++x) {
            const uint8_t* p =
                &resized[(static_cast<size_t>(y) * kInputW + x) * 3];
            out_chw[(0 * kInputH + y) * kInputW + x] = p[2] * inv - 1.0f;  // R
            out_chw[(1 * kInputH + y) * kInputW + x] = p[1] * inv - 1.0f;  // G
            out_chw[(2 * kInputH + y) * kInputW + x] = p[0] * inv - 1.0f;  // B
        }
    }
    return true;
}

}  // namespace inferd
