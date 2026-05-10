from ctypes import (
    POINTER,
    byref,
    c_char_p,
    c_double,
    c_int64,
    c_uint64,
    create_string_buffer,
)

from mlibpy.bind.load_symbols import evaluateFunction

_BUF = 512  # default output buffer size


def lower_case(s: str) -> str:
    """Return s converted to lower case."""
    out = create_string_buffer(_BUF)
    evaluateFunction(
        "LowerCase",
        [c_char_p, c_char_p, c_uint64],
        [s.encode(), out, c_uint64(_BUF)],
        c_uint64,
    )
    return out.value.decode()


def upper_case(s: str) -> str:
    """Return s converted to upper case."""
    out = create_string_buffer(_BUF)
    evaluateFunction(
        "UpperCase",
        [c_char_p, c_char_p, c_uint64],
        [s.encode(), out, c_uint64(_BUF)],
        c_uint64,
    )
    return out.value.decode()


def str_to_double(s: str) -> float:
    """Parse a string as a double."""
    result = c_double(0.0)
    evaluateFunction(
        "StrToDouble",
        [c_char_p, POINTER(c_double)],
        [s.encode(), byref(result)],
        c_uint64,
    )
    return result.value


def double_to_str(v: float) -> str:
    """Convert a double to its string representation."""
    out = create_string_buffer(_BUF)
    evaluateFunction(
        "DoubleToStr",
        [c_double, c_char_p, c_uint64],
        [v, out, c_uint64(_BUF)],
        c_uint64,
    )
    return out.value.decode()


def str_to_int(s: str) -> int:
    """Parse a string as an integer."""
    result = c_int64(0)
    evaluateFunction(
        "StrToInt",
        [c_char_p, POINTER(c_int64)],
        [s.encode(), byref(result)],
        c_uint64,
    )
    return int(result.value)


def str_to_bool(s: str) -> bool:
    """Parse a string as a boolean ("true"/"false" etc.)."""
    result = c_uint64(0)
    evaluateFunction(
        "StrToBool",
        [c_char_p, POINTER(c_uint64)],
        [s.encode(), byref(result)],
        c_uint64,
    )
    return bool(result.value)
