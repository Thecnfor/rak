#!/bin/bash/python3 
# -*- encoding: utf-8 -*-
'''
@File    :   log_test.py
@Time    :   2024/03/15
'''
import logging, os, datetime, time, glob
from logging.handlers import RotatingFileHandler

def logger_file_remove_byday(dir, day=10):
    # print(dir)
    # 删除10天次前的日志
    error_file_list = glob.glob(dir+"/*all.log")
    error_file_list.sort()
    for i in range(len(error_file_list)-day):
        time_file_lists = glob.glob(error_file_list[i][:-7]+"*")
        for file_name in time_file_lists:
            # print(file_name)
            os.remove(file_name)
        

def logger_handler(name:str)->logging.Logger:

    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    cur_path = os.path.abspath(os.path.dirname(__file__))    # 当前文件路径
    
    # 存放目录
    log_dir = os.path.join(cur_path, 'logs')  # log_path为存放日志的路径
    # 若不存在logs文件夹,则自动创建
    if not os.path.exists(log_dir):
        os.mkdir(log_dir)
    # 只保留10天次的日志
    logger_file_remove_byday(log_dir)

    now_time = datetime.datetime.now().strftime('%Y-%m-%d')  # 当前日期格式化
    __all_log_path = os.path.join(log_dir, now_time + "-all" + ".log")  # 收集所有日志信息文件
    __error_log_path = os.path.join(log_dir, now_time + "-error" + ".log")  # 收集错误日志信息文件
    formatter = logging.Formatter('%(asctime)s %(filename)s, line %(lineno)d, %(levelname)s:%(message)s')
    handler_cfgs = [{'type':'file', 'filename': __all_log_path, 'level':logging.INFO, 'formatter': formatter},
                {'type':'file', 'filename':__error_log_path, 'level':logging.ERROR, 'formatter': formatter},
                {'type':'console', 'level':logging.DEBUG, 'formatter': formatter}]
    for handler_cfg in handler_cfgs:
        if handler_cfg['type'] == 'file':
            handler = RotatingFileHandler(filename=handler_cfg['filename'], maxBytes=1 * 1024 * 1024, backupCount=3, encoding='utf-8')
        elif handler_cfg['type'] == 'console':
            handler = logging.StreamHandler()
        handler.setFormatter(handler_cfg['formatter'])
        handler.setLevel(level=handler_cfg['level'])
        logger.addHandler(handler)
    return logger



def loger_test():
    # 创建logger对象
    logger = logging.getLogger('mylogger')
    logger.setLevel(logging.DEBUG)

    cur_path = os.path.abspath(os.path.dirname(__file__))    # 当前文件路径
    
    log_path = os.path.join(cur_path, 'logs')  # log_path为存放日志的路径
    if not os.path.exists(log_path): os.mkdir(log_path)  # 若不存在logs文件夹,则自动创建


    # 创建FileHandler对象
    fh = logging.FileHandler('mylog.log')
    fh.setLevel(logging.DEBUG)

    default_formats = {
        # 终端输出格式
        'color_format': '%(log_color)s %(asctime)s-%(name)s %(filename)s line:%(lineno)d-%(levelname)s-[log]: \n    %(message)s',
        # 日志输出格式
        'log_format': '%(asctime)s %(filename)s line:%(lineno)d-%(levelname)s-[log]: %(message)s'
    }
    # 创建Formatter对象
    # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # formatter = logging.Formatter(default_formats["log_format"], datefmt='%a, %d %b %Y %H:%M:%S')
    formatter = logging.Formatter('%(asctime)s %(filename)s line:%(lineno)d-%(levelname)s:-%(message)s')
    fh.setFormatter(formatter)

    # 将FileHandler对象添加到Logger对象中
    logger.addHandler(fh)
    i = 0
    while True:
        # 记录日志信息
        i += 1
        logger.debug('debug message:' + str(i))
        logger.info('info message')
        logger.warning('warning message')
        logger.error('error message')
        logger.critical('critical message')
        time.sleep(1)

logger = logger_handler("my_logger")

if __name__ == '__main__':

    # logger = logger_handler("my_logger")
    # logger.debug('debug message')
    # logger.info('info message')
    # logger.warning('warning message')
    # logger.error('error message')
    # logger.critical('critical message')
    # logger_file("./logs")
    logger_file_remove_byday("./logs")
    # loger_test()