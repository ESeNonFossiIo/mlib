#ifndef _NUMERIX_PYTHON_STATUS_
#define _NUMERIX_PYTHON_STATUS_

#include "_python/types.h"

/// ----------------------------------------------------------------------------
/// Return the status of the call to the c function
enum class NUMERIXStatus : NUMERIXInt {
    None,    //< Status not set yet
    Success, //< No error occurred
    Failed   //< Generic error found
};

/// ----------------------------------------------------------------------------
/// Try-catch block for c++ exceptions.
#define NUMERIX_TRY try {

/// ----------------------------------------------------------------------------
/// Try-catch block for c++ exceptions.
#define NUMERIX_CATCH                                                                                 \
    }                                                                                              \
    catch (...)                                                                                    \
    {                                                                                              \
    }

#endif // _NUMERIX_PYTHON_STATUS_
