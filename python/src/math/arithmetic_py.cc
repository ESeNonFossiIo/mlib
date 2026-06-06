#include <numerix/core/export.h>
#include <numerix/math/arithmetic.h>

#include "_python/status.h"

#include <vector>

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Pythagoras(const double a, ///< [in]
                                       const double b, ///< [in]
                                       double* result  ///< [out]
)
{
    *result = numerix::pythagoras(a, b);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Argmax(const double* data, ///< [in] array data
                                   const NUMERIXInt n,    ///< [in] length
                                   NUMERIXInt* result     ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = static_cast<NUMERIXInt>(numerix::argmax(v));
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Argmin(const double* data, ///< [in] array data
                                   const NUMERIXInt n,    ///< [in] length
                                   NUMERIXInt* result     ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = static_cast<NUMERIXInt>(numerix::argmin(v));
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Max(const double* data, ///< [in] array data
                                const NUMERIXInt n,    ///< [in] length
                                double* result      ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = numerix::max(v);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Min(const double* data, ///< [in] array data
                                const NUMERIXInt n,    ///< [in] length
                                double* result      ///< [out]
)
{
    const std::vector<double> v(data, data + n);
    *result = numerix::min(v);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
// Output buffer out must be pre-allocated with n elements.
NUMERIX_EXPORT NUMERIXStatus NUMERIX_Normalize(
    const double* data, ///< [in]  array data
    const NUMERIXInt n,    ///< [in]  length
    double* out         ///< [out] normalised values (caller allocates n doubles)
)
{
    const std::vector<double> v(data, data + n);
    const std::vector<double> norm = numerix::normalize(v);
    for (NUMERIXInt i = 0; i < n; ++i)
        out[i] = norm[i];
    return NUMERIXStatus::Success;
}
