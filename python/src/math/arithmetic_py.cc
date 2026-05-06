#include <mlib/core/export.h>
#include <mlib/math/arithmetic.h>

#include "_python/status.h"

#include <vector>

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Pythagoras(
    const double a, ///< [in]
    const double b, ///< [in]
    double*      result ///< [out]
)
{
    *result = mlib::pythagoras(a, b);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Argmax(
    const double* data, ///< [in] array data
    const MLIBInt n,    ///< [in] length
    MLIBInt*      result ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = static_cast<MLIBInt>(mlib::argmax(v));
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Argmin(
    const double* data, ///< [in] array data
    const MLIBInt n,    ///< [in] length
    MLIBInt*      result ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = static_cast<MLIBInt>(mlib::argmin(v));
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Max(
    const double* data, ///< [in] array data
    const MLIBInt n,    ///< [in] length
    double*       result ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = mlib::max(v);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_Min(
    const double* data, ///< [in] array data
    const MLIBInt n,    ///< [in] length
    double*       result ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = mlib::min(v);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
// Output buffer out must be pre-allocated with n elements.
MLIB_EXPORT MLIBStatus MLIB_Normalize(
    const double* data, ///< [in]  array data
    const MLIBInt n,    ///< [in]  length
    double*       out   ///< [out] normalised values (caller allocates n doubles)
)
{
    const std::vector<double> v(data, data + n);
    const std::vector<double> norm = mlib::normalize(v);
    for (MLIBInt i = 0; i < n; ++i)
        out[i] = norm[i];
    return MLIBStatus::Success;
}
