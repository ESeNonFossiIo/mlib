from ctypes import POINTER, byref, c_double, c_uint64

from mlibpy.bind.load_symbols import evaluateFunction


def norm_pdf(x: float) -> float:
    """Standard normal PDF φ(x) = exp(−x²/2) / √(2π)."""
    result = c_double(0.0)
    evaluateFunction(
        "NormPdf",
        [c_double, POINTER(c_double)],
        [x, byref(result)],
        c_uint64,
    )
    return result.value


def norm_cdf(x: float) -> float:
    """Standard normal CDF Φ(x) via Abramowitz & Stegun approximation."""
    result = c_double(0.0)
    evaluateFunction(
        "NormCdf",
        [c_double, POINTER(c_double)],
        [x, byref(result)],
        c_uint64,
    )
    return result.value
