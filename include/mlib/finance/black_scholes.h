#pragma once

#include <mlib/finance/enums.h>

/** \addtogroup finance
 *  @{
 */

namespace mlib
{
  namespace finance
  {

    // @brief Calculate the price of a European call option
    // @details This function calculates the price of a European call option based on the Black-Scholes model.
    // The Black-Scholes model is a mathematical model used for pricing options contracts.
    // It is based on the assumption that the price of the underlying asset follows a geometric Brownian motion
    // with constant volatility and that the risk-free interest rate is known and constant.
    // The model takes into account the current price of the underlying asset, the strike price of the option,
    // the risk-free interest rate, the volatility of the underlying asset, and the time to maturity of the option.
    double BSPrice(
        const double S,          ///< [in] current price of the underlying asset
        const double K,          ///< [in] strike price of the option
        const double r,          ///< [in] risk-free interest rate
        const double v,          ///< [in] volatility of the underlying asset
        const double T,          ///< [in] time to maturity of the option
        const OptionType optType ///< [in] type of option (European call or put)
    );
  }

}

/** @}*/
