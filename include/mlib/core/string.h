#ifndef _MLIB_MACRO_STRING_
#define _MLIB_MACRO_STRING_

/// Convert a macro to a char*
#define TOSTR(x) #x
#define STRVALUE(name) TOSTR(name)

#endif // _MLIB_MACRO_STRING_
