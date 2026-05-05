#ifndef MLIB_MATH_STATISTICS_NORM_DIST_H
#define MLIB_MATH_STATISTICS_NORM_DIST_H

#include "mlib/math/constants.h"

#include <cmath>

/** \addtogroup math
 *  @{
 */

/** \addtogroup statistics
 *  @{
 */

namespace mlib
{

namespace math
{

namespace statistics
{

// ----------------------------------------------------------------------------
// \brief PDF of the standard normal distribution φ(x) = exp(−x²/2) / √(2π).
inline double
norm_pdf(const double x ///< [in] evaluation point
)
{
    return mlib::constants::inv_sqrt_two_pi * std::exp(-0.5 * x * x);
}

// ----------------------------------------------------------------------------
// \brief CDF of the standard normal distribution Φ(x) via Horner-form approximation.
//
// Uses the Abramowitz & Stegun rational approximation (maximum error < 7.5e-8).
// k is evaluated with |x| so that the polynomial argument stays in (0, 1).
// \return Φ(x) in (0, 1)
inline double
norm_cdf(const double x ///< [in] evaluation point
)
{
    const double z = std::fabs(x); // A&S require a positive argument for k
    const double k = 1.0 / (1.0 + 0.2316419 * z);
    const double k_sum =
        k * (0.319381530 +
             k * (-0.356563782 +
                  k * (1.781477937 + k * (-1.821255978 + 1.330274429 * k))));

    const double cdf = 1.0 - (mlib::constants::inv_sqrt_two_pi * std::exp(-0.5 * z * z) * k_sum);

    return (x >= 0.0) ? cdf : 1.0 - cdf;
}

} // namespace statistics

} // namespace math

} // namespace mlib

/** @}*/
/** @}*/

#endif // MLIB_MATH_STATISTICS_NORM_DIST_H
