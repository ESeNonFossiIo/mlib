from ctypes import POINTER, byref, c_char_p, c_uint64

from mlibpy.bind.load_symbols import evaluateFunction


def file_exists(path: str) -> bool:
    """Return True if the file at path exists."""
    result = c_uint64(0)
    evaluateFunction(
        "FileExists",
        [c_char_p, POINTER(c_uint64)],
        [path.encode(), byref(result)],
        c_uint64,
    )
    return bool(result.value)


def get_number_of_lines(path: str) -> int:
    """Return the number of lines in the file at path."""
    result = c_uint64(0)
    evaluateFunction(
        "GetNumberOfLines",
        [c_char_p, POINTER(c_uint64)],
        [path.encode(), byref(result)],
        c_uint64,
    )
    return int(result.value)
