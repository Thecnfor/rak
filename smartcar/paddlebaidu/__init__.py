from .ernie_bot import ErnieBotWrap, HumAttrPrompt, ActionPrompt, ImagePrompt, OrderPrompt
# 注意: 不再在包初始化时导入 paddle_jetson(会拉起整个 paddle 依赖)。
# 运行时推理走 trt_backend(TensorRT); 需要 paddle 时请显式导入子模块。
from .infer_cs import ClintInterface, Bbox