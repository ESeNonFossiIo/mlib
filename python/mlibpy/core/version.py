from mlibpy.bind.load_symbols import evaluateFunction
from mlibpy.bind.types import from_ccharp_to_str
from ctypes import c_char_p, c_uint64

def version():
    """ Return the version of MLIB in the format (major, minor, patch)

        Returns:
            tuple: (major, minor, patch)
    """
    major = evaluateFunction("version_major", [], [], c_uint64)
    minor = evaluateFunction("version_minor", [], [], c_uint64)
    patch = evaluateFunction("version_patch", [], [], c_uint64)

    return (major, minor, patch)
