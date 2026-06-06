from ctypes import POINTER, byref, c_double, c_uint64

from numerixpy.bind.load_symbols import evaluateFunction


def truncate_decimals(num: float, size: float = 10.0) -> float:
    """Truncate num to the precision given by size (e.g. size=10 → 1 decimal place)."""
    result = c_double(0.0)
    evaluateFunction(
        "TruncateDecimals",
        [c_double, c_double, POINTER(c_double)],
        [num, size, byref(result)],
        c_uint64,
    )
    return result.value
