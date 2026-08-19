from .backend import CarBackend, EventType, TASK_ORDER, TASK_INFO
from .mock_backend import MockBackend
from .server import create_app

__all__ = [
    "CarBackend",
    "EventType",
    "TASK_ORDER",
    "TASK_INFO",
    "MockBackend",
    "create_app",
]

try:
    from .real_backend import RealBackend
    __all__.append("RealBackend")
except Exception:
    RealBackend = None
