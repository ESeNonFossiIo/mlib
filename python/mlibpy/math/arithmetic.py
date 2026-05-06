from ctypes import POINTER, byref, c_double, c_uint64
from typing import List

from mlibpy.bind.load_symbols import evaluateFunction


def pythagoras(a: float, b: float) -> float:
    """Return sqrt(a² + b²) without overflow/underflow."""
    result = c_double(0.0)
    evaluateFunction(
        "Pythagoras",
        [c_double, c_double, POINTER(c_double)],
        [a, b, byref(result)],
        c_uint64,
    )
    return result.value


def _vec_args(v: List[float]):
    """Return (c_array, c_uint64_n) for a Python list."""
    arr = (c_double * len(v))(*v)
    return arr, c_uint64(len(v))


def argmax(v: List[float]) -> int:
    """Return the index of the maximum element."""
    arr, n = _vec_args(v)
    result = c_uint64(0)
    evaluateFunction(
        "Argmax",
        [(c_double * len(v)), c_uint64, POINTER(c_uint64)],
        [arr, n, byref(result)],
        c_uint64,
    )
    return int(result.value)


def argmin(v: List[float]) -> int:
    """Return the index of the minimum element."""
    arr, n = _vec_args(v)
    result = c_uint64(0)
    evaluateFunction(
        "Argmin",
        [(c_double * len(v)), c_uint64, POINTER(c_uint64)],
        [arr, n, byref(result)],
        c_uint64,
    )
    return int(result.value)


def max(v: List[float]) -> float:
    """Return the maximum value in v."""
    arr, n = _vec_args(v)
    result = c_double(0.0)
    evaluateFunction(
        "Max",
        [(c_double * len(v)), c_uint64, POINTER(c_double)],
        [arr, n, byref(result)],
        c_uint64,
    )
    return result.value


def min(v: List[float]) -> float:
    """Return the minimum value in v."""
    arr, n = _vec_args(v)
    result = c_double(0.0)
    evaluateFunction(
        "Min",
        [(c_double * len(v)), c_uint64, POINTER(c_double)],
        [arr, n, byref(result)],
        c_uint64,
    )
    return result.value


def normalize(v: List[float]) -> List[float]:
    """Return v normalised to [0, 1]."""
    n = len(v)
    arr = (c_double * n)(*v)
    out = (c_double * n)()
    evaluateFunction(
        "Normalize",
        [(c_double * n), c_uint64, POINTER(c_double)],
        [arr, c_uint64(n), out],
        c_uint64,
    )
    return list(out)
