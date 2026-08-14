from .realtime import RealtimeMixin
from .infer_init import InferInitMixin
from .detect import DetectMixin
from .ocr import OcrErnieMixin


class PerceptionMixin(RealtimeMixin, InferInitMixin, DetectMixin, OcrErnieMixin):
    pass


__all__ = [
    "PerceptionMixin",
    "RealtimeMixin",
    "InferInitMixin",
    "DetectMixin",
    "OcrErnieMixin",
]
