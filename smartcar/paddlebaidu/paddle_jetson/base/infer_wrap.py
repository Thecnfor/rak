import argparse
from typing import List
from paddle.inference import Config, create_predictor

import cv2
import numpy as np
import glob
import time
import os
import sys


# 添加上两层目录
dir_file = os.path.dirname(__file__)
# sys.path.append(os.path.abspath(dir_file))
root_path = os.path.join(dir_file, "..", "..")
# sys.path.append(root_path) 
deploy_path = os.path.abspath(os.path.join(dir_file, "deploy"))
sys.path.append(deploy_path) 
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


from .deploy.python.infer import Detector
from .deploy.pptracking.python.mot_sde_infer import SDE_Detector
from .deploy.pipeline.pphuman.attr_infer import AttrDetector
from .deploy.pipeline.ppvehicle.vehicle_plate import PlateRecognizer, PlateDetector, TextRecognizer
from .deploy.python.utils import nms
# from ...ernie_bot import HumAttrPrompt





def get_current_dir():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return current_dir

class DetectResult:
    def __init__(self, category_id, label, score, bbox, object_id=0) -> None:
        self.class_id = int(category_id)
        self.object_id = int(object_id)
        self.label_name = label
        self.score = float(score)  # score
        self.bbox = bbox.astype(np.int32)
        if self.bbox[0] < 0:
            self.bbox[0] = 0
        if self.bbox[1] < 0:
            self.bbox[1] = 0
        if self.bbox[2] > 639:
            self.bbox[2] = 639
        if self.bbox[3] > 479:
            self.bbox[3] = 479

        # 当前bbox中心
        self.center = [self.bbox[0] + self.bbox[2] / 2, self.bbox[1] + self.bbox[3] / 2]
        # 整个图片中心
        self.middle = [320, 240]

    def pos_from_center(self):
        error_x = self.center[0] - self.middle[0]
        error_y = self.center[1] - self.middle[1]
        return [error_x, error_y]
    
    def get_pos(self):
        error_x = (self.bbox[0] + self.bbox[2]) / 2 - self.middle[0]
        error_y = (self.bbox[1] + self.bbox[3]) / 2 - self.middle[1]
        return [error_x, error_y]
    
    def tolist(self):
        return [self.class_id, self.object_id, self.label_name,self.score] + self.bbox.tolist()
    
    # 归一化结果
    def tolist_nomoralize(self, size):
        mid_x = size[0] / 2
        mid_y = size[1] / 2
        pt_mid = [mid_x, mid_y]
        # 归一化中心值
        normalized_x = float((self.bbox[0] + self.bbox[2]) / 2 - pt_mid[0]) / pt_mid[0]
        normalized_y = float((self.bbox[1] + self.bbox[3]) / 2 - pt_mid[1]) / pt_mid[1]
        normalized_w = float(self.bbox[2] - self.bbox[0]) / pt_mid[0]
        normalized_h = float(self.bbox[3] - self.bbox[1]) / pt_mid[1]
        return [self.class_id, self.object_id, self.label_name, self.score] + [normalized_x, normalized_y, normalized_w, normalized_h]
    
    def pos_from_pos(self, pos):
        error_x = self.center[0] - pos[0]
        error_y = self.center[1] - pos[1]
        return [error_x, error_y]

    def __repr__(self) -> str:
        return self.__str__()
    
    def __str__(self) -> str:
        return "cls_id:{} obj_id:{} label:{} score:{:.3f} bbox:{}".format(
            self.class_id, self.object_id, self.label_name, self.score, self.bbox
        )
    

# 多目标跟踪（MOT）：单目标跟踪（VOT/SOT）、目标检测（detection）、行人重识别（Re-ID）
# ①检测 ②特征提取、运动预测 ③相似度计算 ④数据关联。
# SORT作为一个粗略的框架，核心就是两个算法：卡尔曼滤波和匈牙利匹配。
# DeepSORT的优化主要就是基于匈牙利算法里的这个代价矩阵。它在IOU Match之前做了一次额外的级联匹配，利用了外观特征和马氏距离。
# 
# 
class InferInterface:
    def __init__(self, model_dir: str) -> None:
        self.model_dir = os.path.join(get_current_dir(), '../../models/'+model_dir)
        self.predictor = None
    
    def get_model_path(self):
        model_path  = glob.glob(self.model_dir + "/*.pdmodel")[0]
        params_path = glob.glob(self.model_dir + "/*.pdiparams")[0]
        return model_path, params_path
    
    def get_path_abs(self, path_relative):
        return os.path.join(get_current_dir(), path_relative)
    
    def load_cfg(self):
        yml_path = os.path.join(self.model_dir, "infer_cfg.yml")
        with open(yml_path) as f:
            import yaml
            yml_conf = yaml.safe_load(f)
            self.threshold = yml_conf['draw_threshold']
            self.label_list = yml_conf['label_list']
        
    def __call__(self, *args, **kwds):
        return self.predict(*args, **kwds)
    
    def predict(self, image, normalize_out=False, visualize=False):
        pass
    
    def draw_box(self, image, results:List[DetectResult], threshold=0.5):
        for result in results:
            if result.score > threshold:
                cv2.rectangle(image, (result.bbox[0], result.bbox[1]), (result.bbox[2], result.bbox[3]), (0, 255, 0), 2)
                cv2.putText(image, result.label_name, (result.bbox[0]+10, result.bbox[1] + 10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
    def close(self):
        pass
    
class MotHuman(InferInterface):
    def __init__(self, model_dir='mot_ppyoloe_s_36e_pipeline', run_mode='paddle') -> None:
        # 加载模型文件夹
        super().__init__(model_dir)
        config_path = os.path.join(get_current_dir(), 'deploy/pipeline/config/tracker_config.yml')
        self.mot_predictor = SDE_Detector(
                    model_dir=self.model_dir,
                    tracker_config=config_path,
                    run_mode=run_mode,
                    device="GPU",
                    skip_frame_num=2)
        self.skip_frame = 2
        # 加载模型配置
        super().load_cfg()
        self.frame_id = 0


    # 返回值->list[DetectResult]
    def predict(self, image, normalize_out=False, visualize=False) -> List[DetectResult]:
        frame_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        res = self.mot_predictor.predict_image(
                    [frame_rgb],
                    visual=False,
                    reuse_det_result=False,
                    frame_count=self.frame_id)
        # 预测结果再处理, 特征获取id
        res = self.mot_predictor.predict_image(
                    [frame_rgb],
                    visual=False,
                    reuse_det_result=True,
                    frame_count=self.frame_id)
        # 进行解析
        mot_res = parse_mot_res(res)
        ret = []
        for bbox in mot_res["boxes"]:
            # print(bbox)
            obj_id, cls_id, score, rect = int(bbox[0]), int(bbox[1]), bbox[2], bbox[3:].astype(np.int32)
            res =DetectResult(cls_id, self.label_list[cls_id], score, rect, object_id=obj_id)
            if normalize_out:
                res = res.tolist_nomoralize(image.shape[:2][::-1])
            ret.append(res)
        if visualize:
            self.visualize(image, ret)
        # 更新帧
        self.frame_id += 1
        return ret

        def close(self):
            super().close()

    def visualize(self, image, res):
        for obj_box in res:
            obj_id, cls, label, score, bbox = obj_box.object_id, obj_box.class_id, obj_box.label_name, obj_box.score, obj_box.bbox
            cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.putText(image, label, (int(bbox[0])+10, int(bbox[1])+15), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
            cv2.putText(image, str(obj_id), (int(bbox[0])+10, int(bbox[1])+35), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
            cv2.putText(image, "{:.2f}".format(score), (int(bbox[0])+10, int(bbox[1])+55), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)


class YoloeInfer(InferInterface):
    def __init__(self, model_dir="task_model3", run_mode="paddle") -> None:
        super().__init__(model_dir)
        # print(model_dir, run_mode)
        self.yolo_predictor = Detector(
                    model_dir=self.model_dir,
                    device="GPU", run_mode=run_mode)
        super().load_cfg()
    

    def predict(self, image, normalize_out=False) -> List[DetectResult]:
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        det_res = self.yolo_predictor.predict_image([image_rgb], visual=False)
        det_res = self.yolo_predictor.filter_box(det_res,self.threshold)
        # print(det_res)
        det = nms(det_res["boxes"], len(self.label_list))
        # print(det)
        # 返回值处理
        ret = []
        for bbox in det:
            # print(bbox)
            cls_id, score, rect = int(bbox[0]), bbox[1], bbox[2:].astype(np.int32)
            res = DetectResult(cls_id, self.label_list[cls_id], score, rect)
            if normalize_out:
                res = res.tolist_nomoralize(image.shape[:2][::-1])
            ret.append(res)
        return ret

    def close(self):
        super.close()

class YolovxInfer(InferInterface):
    def __init__(self, model_dir="task_model3", run_mode="paddle") -> None:
        super().__init__(model_dir)
        # print(model_dir, run_mode)
        self.yolo_predictor = Detector(
                    model_dir=self.model_dir,
                    device="GPU", run_mode=run_mode)
        super().load_cfg()
    

    def predict(self, image, normalize_out=False) -> List[DetectResult]:
        # img_resize = cv2.resize(image, (480, 480))
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        det_res = self.yolo_predictor.predict_image([image_rgb], visual=False)
        det_res = self.yolo_predictor.filter_box(det_res,self.threshold)
        # 返回值处理
        ret = []
        for bbox in det_res["boxes"]:
            # print(bbox)
            cls_id, score, rect = int(bbox[0]), bbox[1], bbox[2:].astype(np.int32)
            res = DetectResult(cls_id, self.label_list[cls_id], score, rect)
            if normalize_out:
                res = res.tolist_nomoralize(image.shape[:2][::-1])
            ret.append(res)
        return ret

    def close(self):
        super.close()

class YoloeRInfer(InferInterface):
    def __init__(self, model_dir="task_model3", run_mode="paddle") -> None:
        super().__init__(model_dir)
        # print(model_dir, run_mode)
        self.yolo_predictor = Detector(
                    model_dir=self.model_dir,
                    device="GPU", run_mode=run_mode)
        super().load_cfg()
    

    def predict(self, image, normalize_out=False) -> List[DetectResult]:
        # img_resize = cv2.resize(image, (480, 480))
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        det_res = self.yolo_predictor.predict_image([image_rgb], visual=False)
        det_res = self.yolo_predictor.filter_box(det_res,self.threshold)
        # 返回值处理
        ret = []
        for bbox in det_res["boxes"]:
            print(bbox)
            cls_id, score, rect = int(bbox[0]), bbox[1], bbox[2:].astype(np.int32)
            res = DetectResult(cls_id, self.label_list[cls_id], score, rect)
            if normalize_out:
                res = res.tolist_nomoralize(image.shape[:2][::-1])
            ret.append(res)
        return ret

    def close(self):
        super.close()

def parse_mot_res(input):
    mot_res = []
    boxes, scores, ids = input[0]
    for box, score, i in zip(boxes[0], scores[0], ids[0]):
        xmin, ymin, w, h = box
        res = [i, 0, score, xmin, ymin, xmin + w, ymin + h]
        mot_res.append(res)
    return {'boxes': np.array(mot_res)}

class HummanAtrr(InferInterface):
    def __init__(self, model_dir="PPLCNet_x1_0_person_attribute_945_infer", run_mode='paddle') -> None:
        super().__init__(model_dir)
        self.predictor = AttrDetector(
                    model_dir=self.model_dir,
                    device="GPU", run_mode=run_mode)
        super().load_cfg()

#         # 属性配置，和erniebot获取的设置一致
#         self.attr_json = HumAttrPrompt().json_obj()['properties']
#         # print(self.label_list)
    
#     def predict(self, image, normalize_out=False):
#         image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
#         scores =  self.predictor.predict_image([image_rgb], visual=False)['output'][0]
#         return self.to_attr(scores, self.threshold)
    
#     def to_attr(self, score_arr, threshold_attr):
#         '''
#             'hat':{'type': 'boolean', 'description': '戴帽子真,没戴帽子假'},					
#             'glasses': {"type": 'boolean', 'description': '戴眼镜真,没戴眼镜假'},
#             'sleeve':{'enum': ['Short', 'Long'], 'description': '衣袖长短'},
#             # 'UpperStride', 'UpperLogo', 'UpperPlaid', 'UpperSplice'	有条纹		印有logo/图案	撞色衣服(多种颜色) 格子衫
#             'color_upper':{'enum':['Stride', 'Logo', 'Plaid', 'Splice'], 'description': '上衣衣服颜色'},
#             # 'LowerStripe', 'LowerPattern'		有条纹		印有图像
#             'color_lower':{'enum':['Stripe', 'Pattern'], 'description': '下衣衣服长短'},
#             # 'LongCoat', 长款大衣
#             'clothes_upper':{'enum':['LongCoat'], 'description': '上衣衣服类型'},
#             # 'Trousers', 'Shorts', 'Skirt&Dress'  长裤		短裤 	裙子/连衣裙
#             'clothes_lower':{'enum':['trousers', 'shorts', 'skirt_dress'], 'description': '下衣衣服类型'},
#             'boots':{'type': 'boolean', 'description': '穿着鞋子真,没穿鞋子假'},
#             'bag':{'enum': ['HandBag', 'ShoulderBag', 'Backpack'], 'description': '带着包的类型'},
#             'holding':{'type': 'boolean', 'description': '持有物品为真'},
#             'age':{'enum': ['Old', 'Middle','Young'], 'description': '年龄,小于18岁为young，18到60为middle，大于60为old'},
#             'sex':{'enum': ['Male', 'Female']},
#             'direction':{'enum': ['Front', 'Side', 'Back'], 'description': '人体朝向'},	
        
#         '''
#         dict_attr = self.attr_json
#         # print(dict_attr)
#         require_arr = {'hat':True, 'glasses':True, 'sleeve':False, 'color_upper':False, 'color_lower':False, 
#                        'clothes_upper':False, 'clothes_lower':False, 'boots':True, 'bag':False, 'holding':True, 
#                        'age':True, 'sex':True, 'direction':True}
#         # score_arr = np.array([0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1])
#         # print(score_arr)
#         threshold_attr = 0.5
#         # 'threshold' in attr_tmp
#         index_s = 0
#         index_e = 0
#         ret_dict = {}
#         for key, val in dict_attr.items():
#             # print(key, val)
#             threshold = threshold_attr
#             if 'threshold' in val:
#                  threshold = val['threshold']

#             if key == 'sex':
#                 index_e = index_s + 1
#                 if score_arr[index_s] > threshold:
#                     ret_dict[key] = val['enum'][0]
#                 else:
#                     ret_dict[key] = val['enum'][1]
#             elif key == 'age':
#                 index_e = index_s + len(val['enum'])
#                 # print(index_s, index_e)
#                 # print(score_arr[index_s:index_e])
#                 if score_arr[index_s] > 0.1:
#                     ret_dict[key] = 'Old'
#                 elif score_arr[index_s+2] > 0.1:
#                     ret_dict[key] = 'Young'
#                 else:
#                     ret_dict[key] = 'Middle'
#                 # args_index = np.argmax(np.array(score_arr[index_s:index_e]))
                
#                 # if require_arr[key]:
#                 #     ret_dict[key] = val['enum'][args_index]
#                 # else:
#                 #     if score_arr[index_s+args_index] > threshold:
#                 #         ret_dict[key] = val['enum'][args_index]

#             elif 'type' in val and val['type'] == 'boolean':
#                 index_e = index_s + 1
#                 if score_arr[index_s] > threshold:
#                     ret_dict[key] = True
#                 else:
#                     ret_dict[key] = False
                
#             elif 'enum' in val:
#                 index_e = index_s + len(val['enum'])
#                 # print(index_s, index_e)
#                 args_index = np.argmax(np.array(score_arr[index_s:index_e]))
                
#                 if require_arr[key]:
#                     ret_dict[key] = val['enum'][args_index]
#                 else:
#                     if score_arr[index_s+args_index] > threshold:
#                         ret_dict[key] = val['enum'][args_index]
#             index_s = index_e
#         return ret_dict

#     def close(self):
#         super().close()

def get_rotate_crop_image(img, points):
    '''
    img_height, img_width = img.shape[0:2]
    left = int(np.min(points[:, 0]))
    right = int(np.max(points[:, 0]))
    top = int(np.min(points[:, 1]))
    bottom = int(np.max(points[:, 1]))
    img_crop = img[top:bottom, left:right, :].copy()
    points[:, 0] = points[:, 0] - left
    points[:, 1] = points[:, 1] - top
    '''
    assert len(points) == 4, "shape of points must be 4*2"
    img_crop_width = int(
        max(
            np.linalg.norm(points[0] - points[1]),
            np.linalg.norm(points[2] - points[3])))
    img_crop_height = int(
        max(
            np.linalg.norm(points[0] - points[3]),
            np.linalg.norm(points[1] - points[2])))
    pts_std = np.float32([[0, 0], [img_crop_width, 0],
                          [img_crop_width, img_crop_height],
                          [0, img_crop_height]])
    M = cv2.getPerspectiveTransform(points, pts_std)
    dst_img = cv2.warpPerspective(
        img,
        M, (img_crop_width, img_crop_height),
        borderMode=cv2.BORDER_REPLICATE,
        flags=cv2.INTER_CUBIC)
    dst_img_height, dst_img_width = dst_img.shape[0:2]
    if dst_img_height * 1.0 / dst_img_width >= 1.5:
        dst_img = np.rot90(dst_img)
    return dst_img

class OCRReco(InferInterface):
    def __init__(self, det_model_dir="ch_PP-OCRv3_det_infer", rec_model_dir="ch_PP-OCRv3_rec_infer", run_mode='paddle') -> None:
        parser = argparse.ArgumentParser()
        parser.add_argument('--device', default="GPU", help='foo help')
        parser.add_argument('--run_mode', default=run_mode, help='foo help')
        args = parser.parse_args()
        use_gpu = True
        # print(args.foo)
        current_dir = get_current_dir()
        det_model_dir = current_dir + '/../../models/' + det_model_dir
        rec_model_dir = current_dir + '/../../models/' + rec_model_dir
        cfg = {
            "det_model_dir": det_model_dir,
            "rec_model_dir": rec_model_dir,
            "det_limit_side_len": 736,
            "det_limit_type": "min",
            "rec_image_shape": [3, 48, 320],
            "rec_batch_num": 6,
            "word_dict_path": self.get_path_abs("deploy/pipeline/ppvehicle/rec_word_dict.txt")
        }

        self.platedetector = PlateDetector(args, cfg=cfg)
        args.run_mode = 'paddle'
        self.textrecognizer = TextRecognizer(args, cfg, use_gpu=use_gpu)
        self.threshold = 0.5
        # super().load_cfg()
    
    def predict(self, image, normalize_out=False):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_list = [image_rgb]

        plate_text_list = []
        plateboxes, det_time = self.platedetector.predict_image(image_list)
        for idx, boxes_pcar in enumerate(plateboxes):
            plate_pcar_list = []
            for box in boxes_pcar:
                # 获取中心坐标
                box_cx = np.mean(box[:, 0])
                box_cy = np.mean(box[:, 1])
                # print(box_cx, box_cy)
                plate_images = get_rotate_crop_image(image_list[idx], box)
                plate_texts, rec_time = self.textrecognizer.predict_text([plate_images])
                plate_texts = list(plate_texts[0])
                # print(type(plate_text_list))
                # print(plate_texts)
                plate_pcar_list.append(plate_texts)
                # print(plate_texts)
            plate_text_list.append(plate_pcar_list)

        # print(plate_text_list[0])
        text_res = ""
        for i in range(len(plate_text_list[0])):
            text, score  = plate_text_list[0][-i-1]
            if score > 0.5:
                text_res = text_res + text
            # print(text, score)
        return text_res


class LaneInfer(InferInterface):
    def __init__(self, model_dir="lane_model", run_mode="paddle") -> None:
        super().__init__(model_dir)
        model_path, params_path = self.get_model_path()
        self.config = Config(model_path, params_path)

        self.config.enable_use_gpu(100, 0)
        self.config.switch_ir_optim()
        self.config.enable_memory_optim()
        self.predictor = create_predictor(self.config)

        self.img_size = (128, 128)
        self.mean = np.array([1.0, 1.0, 1.0])

        self.std = None

    # 归一化处理
    def normalize(self, img):
        img = img.astype(np.float32) / 127.5
        if self.mean is not None:
            img -= self.mean
        if self.std is not None:
            img /= self.std
        # img = (img - self.mean) / self.std
        # 为什么上面的前两个可以，后面的那个不行,
        # 下面这个是操作后赋值了新的变量，前面的是变量没有更改
        return img

    def preprocess(self, img):
        # 更改分辨率
        img = cv2.resize(img, self.img_size)
        img = self.normalize(img)
        # img = self.resize(img, self.img_size)
        # bgr-> rgr
        img = img[:, :, ::-1].astype('float32')  # bgr -> rgb
        img = img.transpose((2, 0, 1))  # hwc -> chw
        return img[np.newaxis, :]
    
    def predict(self, img, normalize_out=False):
        # copy img data to input tensor
        img = self.preprocess(img)
        input_names = self.predictor.get_input_names()

        input_tensor = self.predictor.get_input_handle(input_names[0])
        # input_tensor.reshape(img.shape)
        input_tensor.copy_from_cpu(img)
        # do the inference
        self.predictor.run()
        # get out data from output tensor
        output_names = self.predictor.get_output_names()

        output_tensor = self.predictor.get_output_handle(output_names[0])
        output_data = output_tensor.copy_to_cpu()[0]
        if normalize_out:
            output_data = output_data.tolist()
        return output_data

    def close(self):
        super().close()
    
# def human_attr_test():
#     from camera import Camera
#     cap = Camera(0)

#     human_mot = MotHuman()
#     attr_infer = HummanAtrr()
#     time_start = time.time()
#     while True:
#         img = cap.read()
#         if img is not None:
#             humans = human_mot(img)
#             for human in humans:
#                 # print(human.bbox)
#                 cv2.rectangle(img, (human.bbox[0], human.bbox[1]), (human.bbox[2], human.bbox[3]),(0, 255, 0), 2)
#                 human_crop = img[human.bbox[1]:human.bbox[3], human.bbox[0]:human.bbox[2]]
#                 # cv2.imwrite("human_crop.jpg", human_crop)
#                 hum_attr = attr_infer(human_crop)
#                 print(hum_attr)
#                 # cv2写字
#                 for i, (key, value) in enumerate(hum_attr.items()):
#                     cv2.putText(img, "{}:{}".format(key, value), (human.bbox[0], human.bbox[1]+i*15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#                 # cv2.putText(img, "{}".format(hum_attr["age"]), (human.bbox[0], human.bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#                 # print(hum_attr)
#             # response = attr(img)
#             # print(response)
#             cv2.imshow("img", img)
#             key_ord = cv2.waitKey(1)
#             if key_ord == ord('q'):
#                 break
#             fps = 1/(time.time()-time_start)
#             print("fps:", fps)
#             time_start = time.time()
#             # cv2.putText(img, "fps:{}".format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def cam_infer_test():
    from camera import Camera
    cap = Camera(2)
    # infer = LaneInfer()
    # infer = YolovxInfer("yolov10_n")
    # infer = YolovxInfer("yolov8_n_500e_480_coco",run_mode="trt_fp32")
    infer = YoloeInfer("task_wbt2025")
    # infer = YoloeRInfer("ppyoloe_r_crn_s_3x_dota")
    # infer = YoloeInfer("ppyoloe_365obj")
    # infer = HummanAtrr()
    time_start = time.time()
    ocr = OCRReco()
    while True:
        img = cap.read()
        if img is not None:
            # print(img.shape)
            response = infer(img)
            for res in response:
                if res.class_id == 0:
                    x1, y1, w, h = res.bbox
                    x2 = x1 + w
                    x2 = img.shape[1] if x2> img.shape[1] else x2
                    y2 = y1 + h
                    y2 = img.shape[0] if y2> img.shape[0] else y2
                    img_txt = img[y1:y2, x1:x2]
                    text = ocr(img_txt)
                    print(text)
                    # print(res.bbox)
                else:
                    print(res)
            # infer.draw_box(img, response)
            # print(response)
            fps = 1/(time.time()-time_start)
            print("fps:", fps)
            time_start = time.time()
            # cv2.putText(img, "fps:{}".format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # cv2.imshow("img", img)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
def ocr_test():
    ocr = OCRReco()
    img = cv2.imread('name.png')
    text = ocr.predict(img)
    print(text)



if __name__ == '__main__':
    cam_infer_test()
    # human_attr_test()
    # cam_infer_test()
    ocr_test()
    # print(test)
