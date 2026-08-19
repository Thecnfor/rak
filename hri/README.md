# HRI — Human-Robot Interaction (Jetson Orin)

HRI 人机交互界面, 面向 Jetson Orin (JetPack 6.2 / L4T r36.4.0), 使用 **Qt 6 Quick / QML**,
目标显示屏分辨率为 **1024×600**。
当前阶段在 x86_64 开发机上本地预览迭代, 部署到 Orin 时使用 NVIDIA 官方 l4t 镜像。

## 架构概览

```
┌─ 开发机 (x86_64, Ubuntu) ──────────────────────────────┐
│   Docker: hri-preview 容器                              │
│   ├─ 源码通过卷挂载:  .:/workspace  (改代码免重建镜像)    │
│   ├─ entrypoint:  cmake 增量编译 → 前台运行 hri_app     │
│   └─ Qt xcb 窗口 ──X11 socket──▶ 宿主机屏幕 (DISPLAY=:0) │
└────────────────────────────────────────────────────────┘

部署目标: Jetson Orin (aarch64) → NVIDIA l4t-jetpack:r36.4.0
```

**预览方式**: 容器内编译运行 Qt 应用, 窗口通过 X11 直通弹到宿主机屏幕。
**彻底无 VNC / noVNC** — 不经过浏览器, 直接桌面窗口, 所见即所得。

## 技术栈

- **UI**: Qt 6 Quick / QML (qt_add_qml_module 编译进二进制), Basic 样式
- **后端桥接**: C++ `AppController` (QObject), 通过 context property 暴露给 QML
- **页面结构**: StackLayout 多页面 + 底部导航 (主控 / 状态 / 设置)
- **分辨率**: 固定 1024×600, 高 DPI PassThrough 缩放

## 目录结构

```
.
├── CMakeLists.txt               # Qt6 Quick/QML 构建定义 (com.hri.app 模块)
├── src/
│   ├── main.cpp                 # 应用入口 (QGuiApplication + QML 引擎)
│   ├── app/
│   │   ├── AppController.h      # C++ 后端桥接层 (暴露页面/状态给 QML)
│   │   └── AppController.cpp
│   └── qml/
│       ├── main.qml             # 根窗口: 标题栏 + 页面栈 + 底部导航
│       ├── components/
│       │   └── BottomNav.qml    # 主控/状态/设置 底部导航
│       └── pages/
│           ├── HomePage.qml     # 主控台
│           ├── StatusPage.qml   # 状态
│           └── SettingsPage.qml # 设置
├── docker/
│   ├── Dockerfile.preview       # 本地 x86_64 预览容器 (Ubuntu 22.04 + Qt6)
│   ├── Dockerfile.orin          # Orin 部署参考 (aarch64 / nvcr.io l4t)
│   └── entrypoint.preview.sh    # 预览容器入口: 增量编译 + 前台运行
└── docker-compose.yml           # 本地预览编排 (X11 直通, host 网络)
```

## 快速开始 (首次)

```bash
# 宿主机先放行本地 X11 连接 (开发机安全域内可行)
xhost +local:

# 构建镜像 + 启动容器 (首次会自动 build)
docker compose up -d --build

# 确认窗口弹出在屏幕 (标题: HRI — 主控台)
```

依赖项: 宿主已装 `xdg-utils`/X server, Docker, QEMU binfmt (aarch64 参照镜像用, 可选)。
预览容器本身自带 Qt6 完整工具链, 宿主机无需装 Qt。

## 日常迭代 (最快路径)

只改源码, 然后:

```bash
docker compose restart
```

entrypoint 会自动:
1. `cmake -S . -B build -G Ninja`  (增量配置, 不改即 no work)
2. `cmake --build build`           (ninja 只重编改动文件, 秒级)
3. `exec ./build/hri_app`          (前台运行, 窗口立即弹到你屏幕)

**改完存盘 → restart → 1~4 秒看效果**, 这就是当前的标准迭代循环。

> 已实现的提速特性: 源码卷挂载免重建镜像; Ninja 增量编译; X11 直通免开浏览器。

## 扩展新页面 (指南)

以新增一个"地图"页为例:

1. 在 `src/qml/pages/` 新建 `MapPage.qml` (根元素 Rectangle, 透明背景)
2. 在 `CMakeLists.txt` 的 `qt_add_qml_module` 的 `SOURCES` 里追加该文件
3. 在 `main.qml` 的 `StackLayout` 中加一行 `MapPage {}`
4. 在 `components/BottomNav.qml` 的 `tabs` 数组加一项 (图标 + 标题 + 对应索引)
5. 如需要 C++ 数据, 在 `AppController` 暴露属性/槽, QML 直接引用

## 遇到的注意事项

- `MESA: error: Failed to query drm device` / `libGL ... iris` — 容器内无 GPU/DRM
  时的**软渲染告警**, 不影响 QML 显示, 可忽略。后续追求 GPU 加速再优化。
- `redis`/`postgres`/`mqtt` 是本机其他 C4 服务的容器, 与本项目无关。
- 首次 `docker compose up -d --build` 因为装 Qt6 会耗时 ~2min, 仅一次。

## 部署到 Jetson Orin (参考)

- Orin 用官方镜像 `nvcr.io/nvidia/l4t-jetpack:r36.4.0` (aarch64),
  参照镜像 `hri-dev:r36.4.0` 已构建保留。
- 源码直接部署到 Orin, 在 Orin 上编译运行; 若无显示器则需 X11 转发或换显示方案
  (如 `-platform eglfs` 直出屏幕)。
- Orin 端 `nvidia-container-toolkit` 安装待办 (需 sudo)。
