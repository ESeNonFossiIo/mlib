#pragma once

#include <mlib/finance/enums.h>

/** \addtogroup finance
 *  @{
 */

namespace mlib
{

  namespace finance
  {

    // Calculate the European vanilla call price based on
    // underlying S, strike K, risk-free rate r, volatility of
    // underlying sigma and time to maturity T
    double BSPrice(
      const double S, ///< [in] underlying
      const double K, ///< [in] strike
      const double r, ///< [in] risk-free rate
      const double v, ///< [in] volatility of underlying sigma
      const double T, ///< [in] time to maturity
      const OptionType optType ///< [in] option to compute
    );

  }


}


/** @}*/
