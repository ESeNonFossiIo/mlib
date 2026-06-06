#include <numerix/core/export.h>
#include <numerix/math/statistic.h>

#include "_python/status.h"

#include <vector>

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Mean(const double* data, ///< [in]  array data
                                 const NUMERIXInt n,    ///< [in]  length
                                 double* result      ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = numerix::mean(v);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Var(const double* data, ///< [in]  array data
                                const NUMERIXInt n,    ///< [in]  length
                                double* result      ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = numerix::var(v);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Stddev(const double* data, ///< [in]  array data
                                   const NUMERIXInt n,    ///< [in]  length
                                   double* result      ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = numerix::stddev(v);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Moment(const double* data,    ///< [in]  array data
                                   const NUMERIXInt n,       ///< [in]  length
                                   const NUMERIXInt order,   ///< [in]  moment order
                                   const NUMERIXInt central, ///< [in]  1 = central moment, 0 = raw
                                   double* result         ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = numerix::moment(v, static_cast<std::size_t>(order), central != 0);
    return NUMERIXStatus::Success;
}
