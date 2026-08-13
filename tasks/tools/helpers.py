# -*- coding: utf-8 -*-
"""通用辅助函数（从 car.py 拆分而来）。"""
import os
import re
import signal
import time

import psutil


def filter_chinese_letter(text: str) -> str:
    # 正则：汉字 一-鿿 + 大小写字母 a-zA-Z
    res = re.findall(r"[\u4e00-\u9fffa-zA-Z]", text)
    return "".join(res)


def sellect_program(programs, order, win_order):
    """
    选择程序并生成显示字符串

    该函数用于生成程序选择菜单的显示字符串，突出显示当前选中的程序。

    参数:
        programs: 程序列表，包含所有可选择的程序
        order: 当前选中的程序索引
        win_order: 窗口起始索引

    返回:
        str: 生成的显示字符串，包含程序列表和当前选中的程序标记
    """
    dis_str = ""
    start_index = 0

    start_index = order - win_order
    for i, program in enumerate(programs):
        if i < start_index:
            continue

        now = str(program)
        if i == order:
            now = f">>{i + 1}.{now}"
        else:
            now = f"  {i + 1}.{now}"
        if len(now) >= 19:
            now = now[:19]
        else:
            now = now + "\n"
        dis_str += now
        if i - start_index == 4:
            break
    return dis_str


def kill_other_python():
    """
    终止其他Python进程

    该函数用于终止除当前进程外的其他Python进程，以避免进程冲突。

    注意:
        该函数会强制终止其他Python进程，请谨慎使用。
    """

    pid_me = os.getpid()
    # logger.info("my pid ", pid_me, type(pid_me))
    python_processes = []
    for proc in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            if (
                "python" in proc.info["name"].lower()
                and len(proc.info["cmdline"]) > 1
                and len(proc.info["cmdline"][1]) < 30
            ):
                python_processes.append(proc.info)
        # 出现异常的时候捕获 不存在的异常，权限不足的异常， 僵尸进程
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    for process in python_processes:
        # logger.info(f"PID: {process['pid']}, Name: {process['name']}, Cmdline: {process['cmdline']}")
        # logger.info("this", process['pid'], type(process['pid']))
        if int(process["pid"]) != pid_me:
            os.kill(int(process["pid"]), signal.SIGKILL)
            time.sleep(0.3)


def limit(value, value_range):
    """
    限制值在指定范围内

    该函数用于将输入值限制在[-value_range, value_range]范围内。

    参数:
        value: 输入值
        value_range: 范围上限

    返回:
        float: 限制后的值
    """
    return max(min(value, value_range), 0 - value_range)
