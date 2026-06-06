from ctypes import POINTER, byref, c_double, c_uint64
from typing import List

from numerixpy.bind.load_symbols import evaluateFunction


def _vec_args(v: List[float]):
    arr = (c_double * len(v))(*v)
    return arr, c_uint64(len(v))


def mean(v: List[float]) -> float:
    """Return the arithmetic mean of v."""
    arr, n = _vec_args(v)
    result = c_double(0.0)
    evaluateFunction(
        "Mean",
        [(c_double * len(v)), c_uint64, POINTER(c_double)],
        [arr, n, byref(result)],
        c_uint64,
    )
    return result.value


def var(v: List[float]) -> float:
    """Return the variance of v."""
    arr, n = _vec_args(v)
    result = c_double(0.0)
    evaluateFunction(
        "Var",
        [(c_double * len(v)), c_uint64, POINTER(c_double)],
        [arr, n, byref(result)],
        c_uint64,
    )
    return result.value


def stddev(v: List[float]) -> float:
    """Return the standard deviation of v."""
    arr, n = _vec_args(v)
    result = c_double(0.0)
    evaluateFunction(
        "Stddev",
        [(c_double * len(v)), c_uint64, POINTER(c_double)],
        [arr, n, byref(result)],
        c_uint64,
    )
    return result.value


def moment(v: List[float], order: int = 1, central: bool = False) -> float:
    """Return the n-th moment of v."""
    arr, n = _vec_args(v)
    result = c_double(0.0)
    evaluateFunction(
        "Moment",
        [(c_double * len(v)), c_uint64, c_uint64, c_uint64, POINTER(c_double)],
        [arr, n, c_uint64(order), c_uint64(1 if central else 0), byref(result)],
        c_uint64,
    )
    return result.value
