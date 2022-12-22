from ctypes import c_char_p

def from_ccharp_to_str(ptr):
    return c_char_p(ptr).value.decode("utf-8")
