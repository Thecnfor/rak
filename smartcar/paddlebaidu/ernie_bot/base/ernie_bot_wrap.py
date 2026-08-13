# -*- coding: utf-8 -*-
import re
import erniebot, json
from jsonschema import validate
import os
import yaml
import base64
from openai import OpenAI

class PromptJson:
	def __init__(self, rulers) -> None:
		self.rulers_str = '请根据下面的schema描述生成给定格式json,只返回json数据,不要其他内容。'
		self.schema_str = ''
		self.example_str = ''

		self.set_rulers(rulers)
		self.set_scheame(self.json_obj())
		self.set_example(self.example())

	def json_obj(self):
		return '''```{'type':'string'}```'''

	def example(self):
		return '正确的示例如下：'
	
	def __call__(self, *args, **kwargs):
		pass
    
	def set_scheame(self, json_obj):
		# json转字符串去空格,换行，制表符
		json_str = str(json_obj).replace(' ', '').replace('\n', '').replace('\t', '')
		# 加上三个引号
		json_str = '```' + json_str + '```'
		self.schema_str = json_str

	def set_example(self, example_str:str):
		# 去空格,换行，制表符
		example_str = example_str.replace(' ', '').replace('\n', '').replace('\t', '')
		self.example_str = example_str

	def set_rulers(self, rulers):
		self.rulers_str = rulers.replace(' ', '').replace('\n', '').replace('\t', '')

	def __str__(self) -> str:
		return self.__repr__()
	
	def __repr__(self) -> str:
		return self.rulers_str + self.schema_str + self.example_str
	
class ActionPrompt(PromptJson):
	def __init__(self) -> None:
		
		rulers = '''你是一个机器人动作规划者，需要把我的话翻译成机器人动作规划并生成对应的json结果，机器人工作空间参考右手坐标系。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)
		# self.set_rulers(rulers)
		# self.set_scheame(self.json_obj())
		# self.set_example(self.example())

	def json_obj(self)->dict:
		schema_move = {'type': 'object', 'required': ['func', 'x', 'y', 'angle'],
               'porperties':{
                                'func': {'description': '移动', 'const': 'move'},
                                'x': {'description': 'x坐标, 前后移动, 向前移动正值，向后移动负值', 'type': 'number'},
                                'y': {'description': 'y坐标, 左右移动, 向左移动正值，向右移动负值', 'type': 'number'}, 
                                'angle': {'description': '旋转或者转弯角度，右转顺时针负值，左转逆时针正值', 'type': 'number'}
                            }
            }
		schema_beep = { 'type': 'object', 'required': ['func', 'time_dur'],
				'properties': {'func': {'description': '蜂鸣器,需要发声时', 'const': 'beep'}, 
                   'time_dur': {'description': '蜂鸣器发声持续时间', 'type': 'number'}}
		}

		schema_light = { 'type': 'object', 'required': ['func', 'time_dur'],
						'properties': {'func': {'description': '发光,需要照明时', 'const': 'light'}, 
						'time_dur': {'description': '照亮持续时间', 'type': 'number'}}
		}
		schema_actions = {'type': 'array', 'required': ['items'],
                  'items': {'anyOf': [schema_move, schema_beep, schema_light],
                        'minItems': 1
                    }
		}
		return schema_actions
	
	def example(self)->str:
		example = '''正确的示例如下：
					向左移0.1m, 向左转弯85度: ```[{'func': 'move', 'x': 0, 'y': 0.1, 'angle': 85}]```,
					向右移0.2m, 向前0.1m。 ```[{'func': 'move', 'x': 0.1, 'y': -0.2, 'angle': 0}]```,
					向右转弯85度, 向右移0.1m,。 ```[{'func': 'move', 'x': 0, 'y': -0.1, 'angle': -85}]```,
					蜂鸣器发声5秒。 ```[{'func': 'beep', 'time_dur': 5}]```,
					发光5秒。 ```[{'func': 'light', 'time_dur': 5}]```。
				'''
		return example
	
class HumAttrPrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是一个人特征总结程序，需要根据描述把人的特征生成对应的json结果，如果有对应的描述就写入对应位置。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)

	def json_obj(self)->dict:
		'''
		0 = Hat - 帽子:0无1有
		1 = Glasses - 眼镜:0无1有
		2 = ShortSleeve - 短袖
		3 = LongSleeve - 长袖
		4 = UpperStride - 有条纹
		5 = UpperLogo - 印有logo/图案
		6 = UpperPlaid - 撞色衣服(多种颜色)
		7 = UpperSplice - 格子衫
		8 = LowerStripe - 有条纹
		9 = LowerPattern - 印有图像
		10 = LongCoat - 长款大衣
		11 = Trousers - 长裤
		12 = Shorts - 短裤
		13 = Skirt&Dress - 裙子/连衣裙
		14 = boots - 鞋子
		15 = HandBag - 手提包
		16 = ShoulderBag - 单肩包
		17 = Backpack - 背包
		18 = HoldObjectsInFront - 手持物品
		19 = AgeOver60 - 大于60
		20 = Age18-60 - =18~60
		21 = AgeLess18 - 小于18
		22 = Female - 0:男性; 1:女性
		23 = Front - 人体朝前
		24 = Side - 人体朝侧
		25 = Back - 人体朝后
		'''
		schema_attr = {'type': 'object', 
                'properties':{
                    'hat':{'type': 'boolean', 'description': '戴帽子真,没戴帽子假'},
					'glasses': {"type": 'boolean', 'description': '戴眼镜真,没戴眼镜假', 'threshold':0.15},
					'sleeve':{'enum': ['Short', 'Long'], 'description': '衣袖长短'},
					# 'UpperStride', 'UpperLogo', 'UpperPlaid', 'UpperSplice'	有条纹		印有logo/图案	撞色衣服(多种颜色) 格子衫
					'color_upper':{'enum':['Stride', 'Logo', 'Plaid', 'Splice'], 'description': '上衣衣服颜色'},
					# 'LowerStripe', 'LowerPattern'		有条纹		印有图像
					'color_lower':{'enum':['Stripe', 'Pattern'], 'description': '下衣衣服长短'},
					# 'LongCoat', 长款大衣
					'clothes_upper':{'enum':['LongCoat'], 'description': '上衣衣服类型', 'threshold':0.8},
					# 'Trousers', 'Shorts', 'Skirt&Dress'  长裤		短裤 	裙子/连衣裙
					'clothes_lower':{'enum':['Trousers', 'Shorts', 'Skirt_dress'], 'description': '下衣衣服类型'},
					'boots':{'type': 'boolean', 'description': '穿着鞋子真,没穿鞋子假'},
					'bag':{'enum': ['HandBag', 'ShoulderBag', 'Backpack'], 'description': '带着包的类型'},
					'holding':{'type': 'boolean', 'description': '持有物品为真', 'threshold':0.5},
					'age':{'enum': ['Old', 'Middle', 'Young'], 'description': '年龄,小于18岁为young, 18到60为middle, 大于60为old'},
					'sex':{'enum': ['Female', 'Male'], 'threshold':0.6},
					'direction':{'enum': ['Front', 'Side', 'Back'], 'description': '人体朝向'},
					},
                "additionalProperties": False
            }
		return schema_attr
	
	def example(self)->str:
		example = '''正确的示例如下：
					一个带着眼镜的老人: ```{'glasses': True, 'age': 'old'}```,
					一个带着帽子的中年人: ```{'hat': True, 'age': 'middle'}``` ,
					穿着短袖的带着眼镜的人: ```{'glasses': True, 'clothes': 'short'}``` 。
				'''
		return example

class EduCounselerPrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是一个人中小学指导程序，需要根据描述的题目，逐步进行推理，根据给出的选项选择出正确答案的json结果。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)
	
	def json_obj(self)->dict:
		schema_edu = {
						"type": "object", "required": ['answer', 'analysis'],
						"properties": {
							"analysis":{'type':"string", "description":"题目分析的具体过程,分析的过程少于20字"},
							"answer": {'enum': ['A', 'B', 'C', "D"], "description": "答案选项中的一个"}
						},
						"additionalProperties": False
            		 }
		return schema_edu
	
	def example(self)->str:
		example = '''正确的示例如下：
					题目: 1+1=？, 答案选项有: A.4 B.44 C.7 D.2 ```{"description":"1+1的结果是2,其中选项D和答案一致,所以选D",'answer': 'D'}``` ,
					题目: 1+2=？, 答案选项有: A.3 B.44 C.6 D.2: ```{"description":"1+2的结果是3,其中选项A和答案一致,所以选A",'answer': 'A'}``` 。'''
		return example


class OrderPrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是订单处理程序，需要根据订单信息分析订单内容，生成包含订单人名、订单货物、配送地址的订单处理json结果。
					严格按照下面的scheame描述生成给定格式json，只返回json数据:
				'''
		super().__init__(rulers)
	
	def json_obj(self)->dict:
		schema_order = {
						"type": "object", "required": ['name', 'goods', 'address'],
						"properties": {
							"name": {'type':"string", "description": "订单人名"},
							"goods": {'enum': ['青椒', '蘑菇', '芹菜', '番茄','油菜','豆角','西兰花','土豆','金针菇'], "description": "订单中用户需要的货物"},
							"address": {'enum': [1,2], "description": "配送地址为1号楼或2号楼，1单元或2号单元，需要根据订单内容判断配送地址为数字1或2"}
						},
						"additionalProperties": False
            		 }
		return schema_order	
	
	def example(self)->str:
		example = '''正确的示例如下：
					订单信息: 我需要一份豆角，我是王五，家住1号楼。 ```{"name": "王五", "goods": "豆角", "address": 1}``` ,
					订单信息: 2号楼的张三想做西红柿炒鸡蛋，目前家里已经有鸡蛋了，把欠缺的主要食材给他送过去。 ```{"name": "张三", "goods": "番茄", "address": 2}``` 。'''
		return example

class ImagePrompt(PromptJson):
	def __init__(self) -> None:
		rulers = '''你是一个动物识别专家，需要根据输入的图片识别动物种类，并判断该动物是对农田有害动物还是有益动物。
						严格按照下面的scheame描述生成给定格式json，只返回json数据:
						输出要求（严格遵守）：
						- 必须返回一个 JSON 对象，不要包含任何 Markdown 标记（如 ```json）或其他解释文字。
						- JSON 必须包含以下两个字段：
						1. "analysis": (字符串类型) 描述你的分析过程，包括你识别出了什么动物，以及判断它有益/有害的理由。
						2. "result": (整数类型) 如果是有害动物，返回数字 0；如果是有益动物，返回数字 1。

						JSON 格式示例：
						{
							"analysis": "图片中识别到的动物是一只蜜蜂，蜜蜂可以帮助植物传粉，对农作物和生态系统有益。",
							"result": 1
						}
					''' 
		super().__init__(rulers)
	def json_obj(self)->dict:
		schema_edu = {
						"type": "object", "required": ['result', 'analysis'],
						"properties": {
							"result":{'type':'integer', "description":"判断结果，有害动物返回0，有益动物返回1"},
							"analysis":{'type':"string", "description":"分析过程，包括动物识别和有害/有益判断的理由"}
						},
						"additionalProperties": False
					 } 
		return schema_edu
	
	def example(self)->str:
		example = '''正确的示例如下：
						图片中的动物是蜜蜂: ```{"result": 1, "analysis": "蜜蜂是有益动物，能帮助植物授粉"}``` ,
						图片中的动物是老鼠: ```{"result": 0, "analysis": "老鼠是有害动物，会传播疾病和破坏农作物"}``` 。''' 
		return example

class ErnieBotWrap():

	def __init__(self):
		module_dir = os.path.dirname(os.path.abspath(__file__))
		config_path = os.path.join(module_dir, '..', '..', '..', '..', 'config_car.yml')
		with open(config_path, 'r', encoding='utf-8') as f:
			config = yaml.safe_load(f)
			access_token = config['ernie_access_token']

		self.client = OpenAI(
			api_key=access_token,
			base_url="https://aistudio.baidu.com/llm/lmapi/v3",
		)
		self.image_model = "ernie-4.5-turbo-vl"

		erniebot.api_type = 'aistudio'
		erniebot.access_token = access_token

		self.msgs = []
		self.model = 'ernie-4.0'
		self.prompt_str = '请根据下面的描述生成给定格式json'

	@staticmethod
	def get_mes(role, dilog):
		"""获取消息对象"""
		data = {}
		if role == 0:
			data['role'] = 'user'
		elif role ==1:
			data['role'] = 'assistant'
		data['content'] = dilog	
		return data

	def set_promt(self, prompt_str):
		# str_input = prompt_str
		# self.msgs.append(self.get_mes(0, str_input))
		# response = erniebot.ChatCompletion.create(model=self.model, messages=self.msgs, system=prompt_str)
		# str_res = response.get_result()
		# self.msgs.append(self.get_mes(1, str_res))
		# print(str_res)
		# print("设置成功")
		self.prompt_str = prompt_str
		# print(self.prompt_str)
	
	def get_image_res(self, image):
		"""获取图片识别结果"""
		
		# base64_image  = base64.b64encode(image).decode("utf-8")
		base64_image  = image

		system_prompt = '''你是一个动物识别专家，需要根据输入的图片识别动物种类，并判断该动物是对农田有害动物还是有益动物。
						严格按照下面的scheame描述生成给定格式json，只返回json数据:
						输出要求（严格遵守）：
						- 必须返回一个 JSON 对象，不要包含任何 Markdown 标记（如 ```json）或其他解释文字。
						- JSON 必须包含以下两个字段：
						1. "analysis": (字符串类型) 描述你的分析过程，包括你识别出了什么动物，以及判断它有益/有害的理由。
						2. "result": (整数类型) 如果是有害动物，返回数字 0；如果是有益动物，返回数字 1。

						JSON 格式示例：
						{
							"result": 1,
							"analysis": "图片中识别到的动物是一只蜜蜂，蜜蜂可以帮助植物传粉，对农作物和生态系统有益。"
						}
					''' 
		self.set_promt(system_prompt)
		messages=[
            {
                'role': 'user', 'content': [
                    {
                        "type": "text",
                        "text": system_prompt
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/png;base64,{base64_image}"
                        }
                    }
                ]   
            }
        ]
		my_json_schema = {
			"type": "json_schema",  # 固定值
			"json_schema": {
				"name": "animal_schema", # 给你的 Schema 起个名字
				# "strict": True,            # 【重要】设为 True，强制模型严格遵守
				"schema": {
					# 这里写标准的 JSON Schema 定义
					"type": "object",
					"properties": { 							
						"analysis":{'type':"string", "description":"动物识别的分析过程，包括动物识别和有害/有益判断的理由"},
						"result": {'type':'integer', "description": "判断结果，有害动物返回0，有益动物返回1"}
					},
					"required": [ "analysis", "result" ],
					            # 可以加上这个，防止额外字段（百度可能支持）
            		"additionalProperties": False 
				}
			}
		}

		response = self.client.chat.completions.create(
			# model="ernie-4.5-8k-preview",
			model=self.image_model,
			messages=messages,
			top_p=0.1,
			)
		content = response.choices[0].message.content
		data = json.loads(content)
		analysis = data["analysis"]
		result = data["result"]

		return result,analysis

	def get_res(self, str_input, record=False, request_timeout=5):
		if len(str_input)<1:
			return False, None
		start_str = " ```"
		end_str = " ```, 根据这段描述生成给定格式json"
		str_input = start_str + str_input + end_str
		msg_tmp = self.get_mes(0, str_input)
		if record:
			self.msgs.append(msg_tmp)
			msgs = self.msgs
		else:
			msgs = [msg_tmp]
		# Create a chat completion
		try:
			# print(msgs)
			# print("-----------------")
			# print(self.prompt_str)
			response = erniebot.ChatCompletion.create(model=self.model, messages=msgs, system=self.prompt_str, top_p=0.1,
											_config_=dict(api_type="AISTUDIO",), request_timeout=request_timeout)
			# print(response)
		except Exception as e:
			# print(e)
			return False, None
		# _config_=dict(api_type="QIANFAN",)
		# _config_=dict(api_type="AISTUDIO",)
		# print(response)
		str_res = response.get_result()
		if record:
			self.msgs.append(self.get_mes(1, str_res))
		return True, str_res
	

	
	
	
	@staticmethod
	def get_json_str(json_str:str):
		try:
			index_s = json_str.find("```json")
			if index_s == -1:
				index_s = json_str.find("```") 
				if index_s == -1:
					return None
				else:
					index_s += 3
					
			else:
				index_s += 7
			# print(json_str[index_s:])
			index_e = json_str[index_s:].find("```") + index_s
			if index_e == -1:
				return None
			# json_str = json_str[index_s:index_e]
			# print(json_str[index_s:index_e])
			# print(index_s, index_e)
			json_str = json_str[index_s:index_e]
			# 找到注释内容并删除
			json_str.replace("\n", "")
			# print(json_str)
			msg_json = json.loads(json_str)
			return msg_json
			# print(index_s)
			# return json_str
		except Exception as e:
			# print(e)
			return json_str
			'''
			try:
				index_s = json_str.find("```json") + 7
				# index_s = json_str.find("```json") + 7
			except Exception as e:
				index_s = 0
			try:
				index_e = json_str[index_s:].find("```") + index_s
			except Exception as e:
				index_e = len(json_str)
			import json
			msg_json = json.loads(json_str[index_s:index_e])
			return msg_json
			'''
	
	def get_res_json(self, str_input, record=False, request_timeout=10):
		state, str_res = self.get_res(str_input, record, request_timeout)
		if state:
			# print(str_res)
			obj_json = self.get_json_str(str_res)
			return obj_json
		else:
			return None

def test():
	res = '''```json\n[\n  {\n    "func": "my_light",\n    "count": 3\n  },\n  {\n    "func": "beep",\n    "time_dur": 3  // 假设蜂鸣器持续发声3秒作为紧急警示，具体时长可根据实际情况调整\n  }\n]\n```'''
	json_test = ErnieBotWrap.get_json_str(res)
	print(json_test)



if __name__ == "__main__":
	# test()
	# str_input = ''' 如果买满200元可优惠40元。购买了3件商品,价格分别为85元、130元和115元。最终支付多少钱?	选项有: A.300元 B.180元 C.130元 D.290元'''
	str_input = ''' 2号楼的李四要做芹菜炒肉，他现在需要芹菜。 '''
	
	ernie = ErnieBotWrap()
	# 设置prompt
	# ernie.set_promt(str(ImagePrompt()))
	# ernie.set_promt(str(ActionPrompt()))
	# ernie.set_promt(str(HumAttrPrompt()))
	# ernie.set_promt(str(EduCounselerPrompt()))
	ernie.set_promt(str(OrderPrompt()))
	
	# 测试图片分析功能
	# 请将下面的路径替换为实际的动物图片路径
	# image_path = r"C:\Users\mengc\OneDrive\WhalesBot\2026smartcar\code\baidu_smartcar_2026\smartcar\paddlebaidu\ernie_bot\base\image.png"
	# with open(image_path, "rb") as image_file:
	# 	image  = image_file.read()
	# 	base64_image  = base64.b64encode(image).decode("utf-8")
	
	# response = ernie.get_image_res(base64_image)
	# print("-----------------")
	# print(response)
	# print("-----------------")
	# print(response.choices[0].message.content)
	
	# 测试文本输入
	json_res = ernie.get_res_json(str_input)
	print("-----------------")
	print(json_res)
	print(json_res['name'])
	# while True:
	# 	print("用户")
	# 	str_tmp = input("输入:")
	# 	if len(str_tmp)<1:
	# 		continue
	# 	# Create a chat completion
	# 	print("文心一言")
	# 	# _, str_res = ernie.get_res(str_tmp)
	# 	json_res = ernie.get_res_json(str_tmp)
	# 	print("输出:",json_res)
	
