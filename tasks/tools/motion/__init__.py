from .base import MoveMixin
from .lane import LaneMixin
from .locate import LocateMixin


class MotionMixin(LaneMixin, LocateMixin, MoveMixin):
    pass


__all__ = ["MotionMixin", "MoveMixin", "LaneMixin", "LocateMixin"]
