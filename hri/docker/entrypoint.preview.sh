#!/usr/bin/env bash
# HRI 预览容器: 窗口直接 X11 直通到宿主机显示, 不经过 VNC/noVNC
set -euo pipefail

echo "=============================================================="
echo "  HRI 预览 (X11 直通): 窗口将直接显示在宿主机 DISPLAY=$DISPLAY 上"
echo "=============================================================="

# 构建并启动 HRI 应用
cd /workspace
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# 前台运行, Ctrl-C 直接退出容器
exec ./build/hri_app
