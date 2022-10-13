#pragma once

#include <mlib/math/constants.h>

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

/// ----------------------------------------------------------------------------
      /// PDF of standard normal distribution
      inline double norm_pdf(const double x ///< [in] evaluation point
                            )
      {
        return mlib::constants::inv_sqrt_two_pi * exp(-0.5*x*x);
      }

/// ----------------------------------------------------------------------------
      /// CDF of standard normal distribution
      inline double norm_cdf(const double x)
      {
        const double k = 1.0/(1.0 + 0.2316419*x);
        const double k_sum = k*(0.319381530 + k*(-0.356563782 + k*(1.781477937 + k*
                                                                   (-1.821255978 + 1.330274429*k))));

        const double z = std::fabs(x);
        const double cdf = (1.0 - ((mlib::constants::inv_sqrt_two_pi)*exp(
                                     -0.5*z*z) * k_sum));
        if(x >= 0.0)
          {
            return cdf;
          }
        else
          {
            return 1.0 - cdf;
          }
      }


    }

  }

}

/** @}*/

/** @}*/
