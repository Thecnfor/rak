# -*- coding: utf-8 -*-
"""TensorRT 张量 I/O 的 CUDA 封装(基于 libcudart runtime API)。

为什么用 runtime API(cudart)而不是 driver API(cuMemAlloc):
  Jetson Orin 的 nvgpu 驱动(540)上, driver API 的 cuMemAlloc 无论在任何
  上下文里都返回 CUDA_ERROR_INVALID_CONTEXT (201); 而 runtime API 的
  cudaMalloc/cudaMemcpy 工作正常, 且不需要手动创建/切换上下文
  (隐式 primary context, 与 TensorRT 兼容)。

只依赖 libcudart.so: cudaMalloc / cudaFree / cudaMemcpy /
cudaStreamCreate / cudaStreamSynchronize。
"""

import ctypes
import threading

import numpy as np

_LIB = ctypes.CDLL("libcudart.so")

_LIB.cudaMalloc.restype = ctypes.c_int
_LIB.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]

_LIB.cudaFree.restype = ctypes.c_int
_LIB.cudaFree.argtypes = [ctypes.c_void_p]

_LIB.cudaMemcpy.restype = ctypes.c_int
_LIB.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                            ctypes.c_size_t, ctypes.c_int]

_LIB.cudaStreamCreate.restype = ctypes.c_int
_LIB.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]

_LIB.cudaStreamSynchronize.restype = ctypes.c_int
_LIB.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]

_LIB.cudaGetErrorString.restype = ctypes.c_char_p
_LIB.cudaGetErrorString.argtypes = [ctypes.c_int]

# cudaMemcpyKind
_HOST_TO_DEVICE = 1
_DEVICE_TO_HOST = 2


class CudaError(RuntimeError):
    pass


def _check(ret, what):
    if ret != 0:
        detail = _LIB.cudaGetErrorString(ret)
        raise CudaError(f"{what} failed with CUDA error {ret}: "
                        f"{detail.decode() if detail else ''}")


def mem_alloc(nbytes):
    ptr = ctypes.c_void_p()
    _check(_LIB.cudaMalloc(ctypes.byref(ptr), int(nbytes)), "cudaMalloc")
    return int(ptr.value or 0)


def mem_free(ptr):
    if not ptr:
        return
    _check(_LIB.cudaFree(ctypes.c_void_p(ptr)), "cudaFree")


def memcpy_htod(ptr, arr):
    """把 C 连续 numpy 数组拷贝到设备内存。"""
    nbytes = int(arr.size * arr.itemsize)
    _check(_LIB.cudaMemcpy(ctypes.c_void_p(ptr),
                           ctypes.c_void_p(arr.ctypes.data),
                           nbytes, _HOST_TO_DEVICE), "cudaMemcpy(HtoD)")


def memcpy_dtoh(arr, ptr, nbytes=None):
    """把设备内存拷贝到已分配的 numpy 数组。"""
    if nbytes is None:
        nbytes = int(arr.size * arr.itemsize)
    _check(_LIB.cudaMemcpy(ctypes.c_void_p(arr.ctypes.data),
                           ctypes.c_void_p(ptr),
                           nbytes, _DEVICE_TO_HOST), "cudaMemcpy(DtoH)")


_streams = {}


def stream(thread_ident):
    """每线程一个 CUDA stream, 返回 cudaStream_t 句柄(int)。"""
    if thread_ident not in _streams:
        s = ctypes.c_void_p()
        _check(_LIB.cudaStreamCreate(ctypes.byref(s)), "cudaStreamCreate")
        _streams[thread_ident] = int(s.value or 0)
    return _streams[thread_ident]


def stream_sync(stream_handle):
    _check(_LIB.cudaStreamSynchronize(ctypes.c_void_p(
        stream_handle)), "cudaStreamSynchronize")