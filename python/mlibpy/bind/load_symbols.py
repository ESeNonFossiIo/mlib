from ctypes import cdll
from os.path import dirname, abspath, join

# ------------------------------------------------------------------------------
def loadMLIB():
    """ Use ctypes to load c functions
    """

    if not hasattr(loadMLIB, "mlib"):
        # path to the folde that contains this file
        current_dir = dirname(abspath(__file__))
        # path to the folder that contains the library
        lib_dir = join(current_dir, "../../lib")
        # name of the library
        mlib_lib = r"lib_mlib.dylib"

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
