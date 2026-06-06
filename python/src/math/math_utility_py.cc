#include <numerix/core/export.h>
#include <numerix/math/utility.h>

#include "_python/status.h"

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_TruncateDecimals(
    const double num,  ///< [in]  input value
    const double size, ///< [in]  rounding factor (e.g. 10.0 for 1 decimal place)
    double* result     ///< [out]
)
{
    *result = numerix::truncate_decimals(num, size);
    return NUMERIXStatus::Success;
}
