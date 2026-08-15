# --*-- coding: utf-8 --*--
# infer_back_end.py

import zmq
import json
import cv2
import yaml
import numpy as np
from threading import Thread
import time
import os
import sys

# 常驻推理后端(systemd 守护), 必须零硬件依赖:
# 不走 smartcar 包导入, 否则触发 smartcar/__init__ 构造整个硬件栈,
# 会再次打开 /dev/ttyUSB0 与主程序(run.py)抢串口、偷 MC602 应答帧。
_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 以顶层包方式导入 trt_backend(它只依赖 cv2/numpy/tensorrt, 无 smartcar 依赖)
sys.path.insert(0, os.path.join(_BASE_DIR, "..", ".."))
# 项目根目录(读 config_car.yml 用)
_REPO_ROOT = os.path.abspath(os.path.join(_BASE_DIR, "..", "..", "..", ".."))
sys.path.insert(0, _REPO_ROOT)

from trt_backend import TrtYoloeInfer, TrtLaneInfer, TrtCorrectionInfer


def get_path_relative(*args):
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)


def get_yaml(path):
    config_path = os.path.join(_REPO_ROOT, "config_car.yml")
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.load(f, Loader=yaml.FullLoader)
    except Exception as e:
        print('{} not found'.format(config_path))
        print(e)
        return None

class InferServer:
    def __init__(self):
        # 导入推理客户端的配置
        # configs = ClintInterface.configs
        configs = get_yaml('config_car.yml')['infer_cfg']
        
        self.flag_infer_initok = False
    
        self.flag_end = False
        # 开启对应的线程和服务
        self.threads_list = []
        self.server_dict = {}
        
        # self.lane_server = self.get_server(5001)
        for conf in configs:
            print(conf)
            # 创建获取zmq服务
            server = self.get_server(conf['port'])
            self.server_dict[conf['name']] = server
            # 创建线程
            # thread_tmp = Thread(target=eval('self.'+conf['name']+'_process'))
            # 带参数线程，此处参数为各种推理模型
            thread_tmp = Thread(target=self.process_demo, args=(conf['name'],))
            # thread_tmp = Thread(target=self.lane_process)
            thread_tmp.daemon = True
            thread_tmp.start()
            # 添加进程
            self.threads_list.append(thread_tmp)
        
        InferFactory = {
            "YoloeInfer": TrtYoloeInfer,
            "LaneInfer": TrtLaneInfer,
            "CorrectionInfer": TrtCorrectionInfer,
        }
        # 创建推理模型
        self.infer_dict = {}

        for conf in configs:
            InferType = InferFactory[conf['infer_type']]
            if 'model_dir' in conf:
                infer = InferType(conf['model_dir'], run_mode=conf['run_mode'])
            else:
                infer = InferType(run_mode=conf['run_mode'])
            self.infer_dict[conf['name']] = infer

        # 创建推理模型
        # self.lane_infer = LaneInfer()
        # self.front_infer = YoloInfer("front_model2") # "trt_fp32")
        # self.task_infer = YoloInfer("task_model3") # "trt_fp32")
        # self.ocr_infer = OCRReco()
        # self.humattr_infer = HummanAtrr()
        # self.mot_infer = MotHuman()
        
        # 新建一个空白图片，用于预先图片推理
        img = np.zeros((240, 240, 3), np.uint8)
        # 预加载推理几张图片，刚开始推理时速度慢，会有卡顿
        for i in range(3):
            for conf in configs:
                infer_tmp = self.infer_dict[conf['name']]
                infer_tmp(img)
        print("infer init ok")

        self.flag_infer_initok = True


    def get_server(self, port):
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind(f"tcp://127.0.0.1:{port}")
        return socket
    
    def process_demo(self, name):
        
        print(time.strftime("%Y-%m-%d %H:%M:%S"), "{} process start".format(name))
        server:zmq.Socket = self.server_dict[name]
        # lambda定义推理函数，含有归一化处理参数为True, 此处定义方便后续调用
        func = lambda x: self.infer_dict[name](x, True)

        while True:
            if self.flag_end:
                return
            response = server.recv()

            head = response[:5]
            res = []
            if head == b"ATATA":
                if self.flag_infer_initok:
                    res = True
                else:
                    res = False
            elif head == b"image":
                # 把bytes转为jpg格式
                img = cv2.imdecode(np.frombuffer(response[5:], dtype=np.uint8), 1)
                if self.flag_infer_initok:
                    # res = self.lane_infer(img).tolist()
                    # lambda函数
                    res = func(img)
                    
            json_data = json.dumps(res)
            json_data = bytes(json_data, encoding='utf-8')
            server.send(json_data)

    def close(self):
        print("closing...")
        self.flag_end = True
        for thread in self.threads_list:
            # 等待结束
            thread.join()
            # 关闭
            thread.close()

def main():
    print("infer_back_end.py 程序开始运行")
    infer_back = InferServer()

    while True:
        try:
            time.sleep(1)
        except Exception as e:
            print(e)
            break
    time.sleep(0.1)
    infer_back.close()

if __name__ == "__main__":
    main()
