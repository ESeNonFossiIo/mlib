from ctypes import cdll
from os.path import dirname, abspath, join, isfile
from sys import platform

# ------------------------------------------------------------------------------
def loadMLIB():
    """ Use ctypes to load c functions
    """

    if not hasattr(loadMLIB, "mlib"):
        # path to the folde that contains this file
        current_dir = dirname(abspath(__file__))
        # path to the folder that contains the library
        lib_dir = join(current_dir, "../../lib")

        lib_ext = ""
        if platform == "linux" or platform == "linux2":
            # linux
            lib_ext = "so"
        elif platform == "darwin":
            # OS X
            lib_ext = "dylib"
        elif platform == "win32":
            # Windows...
            lib_ext = "dll"

        # Debug builds use a "_d" postfix (set by cmake/macro/debug_flags.cmake).
        # Try the debug variant first so both build types work transparently.
        # TODO: consider using a separate environment variable to specify the library path, which would be more robust and flexible.    
        for candidate in ["lib_mlib_bindings_d." + lib_ext, "lib_mlib_bindings." + lib_ext]:
            if isfile(join(lib_dir, candidate)):
                mlib_lib = candidate
                break
        else:
            mlib_lib = "lib_mlib_bindings." + lib_ext

        # get the library
        loadMLIB.mlib = cdll.LoadLibrary(join(lib_dir, mlib_lib))

    return loadMLIB.mlib

# ------------------------------------------------------------------------------
def evaluateFunction(name, arg_types, arg_values, ret_type=None):
    """ Evaluate a function contained in the dll
        Args:
            name(str): name of the function to evaluate
            arg_types(list): list of types of the arguments
            arg_values(list): list of arguments
            ret_type(ctype): return type (default is None)
    """

    fn = getattr(loadMLIB(), "MLIB_" + name)

    if ret_type:
        fn.restype = ret_type

    fn.argtypes = arg_types

    return fn(*arg_values)
