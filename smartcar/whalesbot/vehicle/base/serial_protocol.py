#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""MC 串口帧协议层: 帧头尾常量与编解码纯函数。

MC601/MC602 共用线路帧格式:
    发送帧: 77 68 <len> <payload...> 0A   (len = len(payload) + 4, 整帧长度含头尾)
    应答帧: 77 68 <total_len> <payload...> 0A
本模块无任何硬件/外部依赖, 可独立单测。
"""
import sys
import os

# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

# 帧头/帧尾
MC_HEADER = bytes.fromhex('77 68')
MC_TAIL = bytes.fromhex('0A')


def pack_mc_frame(cmd: bytes) -> bytes:
    """把内部命令字节打包为线路帧: 77 68 <len> <cmd> 0A (len = len(cmd)+4)。"""
    cmd_len = (len(cmd) + 4).to_bytes(1, 'big')
    return MC_HEADER + cmd_len + cmd + MC_TAIL


def parse_mc_stream(rx: bytes, start: int) -> tuple:
    """从字节流 rx 的 start 起切出一整帧 MC602 帧。
    返回 (完整帧 payload bytes 或 None, 新的消费偏移)。
    帧头不足/长度不足都返回 None(等待更多字节, 不移动偏移)。
    MC602 帧: 77 68 <total_len> <payload...> 0A, 其中 total_len 为整帧长度(含头尾)。
    """
    n = len(rx)
    if n - start < 3:
        return None, start
    if rx[start] != MC_HEADER[0] or rx[start + 1] != MC_HEADER[1]:
        # 头不对: 跳过脏字节, 避免状态机卡死
        return None, start + 1
    total = rx[start + 2]
    if total < 4 or n - start < total:
        return None, start
    frame = rx[start:start + total]
    if frame[-1] != MC_TAIL[0]:
        # 尾不对: 丢弃该"帧头"继续扫描
        return None, start + 1
    # 返回 payload(不含 77 68 len 与 0A)
    return frame[3:-1], start + total
