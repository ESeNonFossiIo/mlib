#ifndef _MLIB_PYTHON_STATUS_
#define _MLIB_PYTHON_STATUS_

#include "_python/types.h"

/// ----------------------------------------------------------------------------
/// Return the status of the call to the c function
enum class MLIBStatus : MLIBInt {
    None,    //< Status not set yet
    Success, //< No error occurred
    Failed   //< Generic error found
};

/// ----------------------------------------------------------------------------
/// Try-catch block for c++ exceptions.
#define MLIB_TRY try {

/// ----------------------------------------------------------------------------
/// Try-catch block for c++ exceptions.
#define MLIB_CATCH                                                                                 \
    }                                                                                              \
    catch (...)                                                                                    \
    {                                                                                              \
    }

#endif // _MLIB_PYTHON_STATUS_
