#include <numerix/core/export.h>
#include <numerix/math/statistics/norm_dist.h>

#include "_python/status.h"

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_NormPdf(const double x, ///< [in]  evaluation point
                                    double* result  ///< [out] φ(x)
)
{
    *result = numerix::math::statistics::norm_pdf(x);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_NormCdf(const double x, ///< [in]  evaluation point
                                    double* result  ///< [out] Φ(x)
)
{
    *result = numerix::math::statistics::norm_cdf(x);
    return NUMERIXStatus::Success;
}
