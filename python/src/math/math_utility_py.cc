#include <mlib/core/export.h>
#include <mlib/math/utility.h>

#include "_python/status.h"

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_TruncateDecimals(
    const double num,    ///< [in]  input value
    const double size,   ///< [in]  rounding factor (e.g. 10.0 for 1 decimal place)
    double*      result  ///< [out]
)
{
    *result = mlib::truncate_decimals(num, size);
    return MLIBStatus::Success;
}
