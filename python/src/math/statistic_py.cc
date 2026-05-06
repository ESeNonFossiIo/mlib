#include <mlib/core/export.h>
#include <mlib/math/statistic.h>

#include "_python/status.h"

#include <vector>

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Mean(
    const double* data, ///< [in]  array data
    const MLIBInt n,    ///< [in]  length
    double*       result ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = mlib::mean(v);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Var(
    const double* data, ///< [in]  array data
    const MLIBInt n,    ///< [in]  length
    double*       result ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = mlib::var(v);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Stddev(
    const double* data, ///< [in]  array data
    const MLIBInt n,    ///< [in]  length
    double*       result ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = mlib::stddev(v);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Moment(
    const double* data,    ///< [in]  array data
    const MLIBInt n,       ///< [in]  length
    const MLIBInt order,   ///< [in]  moment order
    const MLIBInt central, ///< [in]  1 = central moment, 0 = raw
    double*       result   ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = mlib::moment(v, static_cast<std::size_t>(order), central != 0);
    return MLIBStatus::Success;
}
