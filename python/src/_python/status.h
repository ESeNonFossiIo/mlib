#ifndef _MLIB_PYTHON_STATUS_
#define _MLIB_PYTHON_STATUS_

#include "_python/types.h"

/// ----------------------------------------------------------------------------
/// return the status of the call to the c function
enum class MLIBStatus : MLIBInt
{
  None, //< Status not set yet
  Success, //< No error occurred
  Failed //< Generic error found
}

#define MLIB_TRY \
  try{

#define MLIB_CATCH \
} catch (...) {    \
}


#endif // _MLIB_PYTHON_STATUS_
