#include <mlib/core/export.h>
#include <mlib/math/statistics/norm_dist.h>

#include "_python/status.h"

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_NormPdf(
    const double x,      ///< [in]  evaluation point
    double*      result  ///< [out] φ(x)
)
{
    *result = mlib::math::statistics::norm_pdf(x);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_NormCdf(
    const double x,      ///< [in]  evaluation point
    double*      result  ///< [out] Φ(x)
)
{
    *result = mlib::math::statistics::norm_cdf(x);
    return MLIBStatus::Success;
}
