#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imgaug 0.4.0 兼容 numpy 2.x 的幂等修复脚本。

背景:imgaug 0.4.0(2020 年,上游已不维护)在导入时使用 np.sctypes,
而 numpy >= 2.0 移除了该属性,导致 `import imgaug.augmenters` 直接报错:
    AttributeError: `np.sctypes` was removed in the NumPy 2.0 release.
本脚本把 imgaug/imgaug.py 中的三行 np.sctypes 替换为显式 dtype 集合,
重复执行无副作用(检测到已修复则直接跳过)。

用法(装完 requirements.txt 后执行):
    python fix_imgaug_numpy2.py
"""

import os
import site
import sys

IMGAUG_LINES = [
    "NP_FLOAT_TYPES = set(np.sctypes[\"float\"])",
    "NP_INT_TYPES = set(np.sctypes[\"int\"])",
    "NP_UINT_TYPES = set(np.sctypes[\"uint\"])",
]

REPLACEMENT = """try:
    NP_FLOAT_TYPES = set(np.sctypes["float"])
    NP_INT_TYPES = set(np.sctypes["int"])
    NP_UINT_TYPES = set(np.sctypes["uint"])
except AttributeError:  # numpy >= 2.0 removed np.sctypes
    NP_FLOAT_TYPES = {np.float16, np.float32, np.float64}
    NP_UINT_TYPES = {np.uint8, np.uint16, np.uint32, np.uint64}
    NP_INT_TYPES = {np.int8, np.int16, np.int32, np.int64}
"""


def find_imgaug_file():
    """返回已安装 imgaug 的 imgaug.py 路径;找不到则返回 None。"""
    for base in site.getsitepackages() + [site.getusersitepackages()]:
        candidate = os.path.join(base, "imgaug", "imgaug.py")
        if os.path.exists(candidate):
            return candidate
    return None


def main():
    imgaug_path = find_imgaug_file()
    if imgaug_path is None:
        print("未找到已安装的 imgaug,请先执行: pip install -r requirements.txt")
        sys.exit(1)

    with open(imgaug_path, "r", encoding="utf-8") as f:
        content = f.read()

    if "except AttributeError:  # numpy >= 2.0 removed np.sctypes" in content:
        print("imgaug 已修复,无需处理:", imgaug_path)
        return

    replaced = 0
    for line in IMGAUG_LINES:
        if line in content:
            content = content.replace(line, REPLACEMENT)
            replaced += 1

    if replaced == 0:
        print("未在 imgaug 中找到 np.sctypes,跳过:", imgaug_path)
        return

    with open(imgaug_path, "w", encoding="utf-8") as f:
        f.write(content)
    print("已修复 imgaug(numpy 2.x 兼容):", imgaug_path)


if __name__ == "__main__":
    main()