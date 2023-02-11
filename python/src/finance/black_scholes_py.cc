#include <mlib/core/export.h>
#include <mlib/finance/black_scholes.h>

#include "_python/types.h"
#include "_python/status.h"

/// ----------------------------------------------------------------------------
// Calculate the European vanilla call price based on
// underlying S, strike K, risk-free rate r, volatility of
// underlying sigma and time to maturity T
MLIB_EXPORT MLIBStatus MLIB_BSPricer(
  const double S, ///< [in] underlying
  const double K, ///< [in] strike
  const double r, ///< [in] risk-free rate
  const double v, ///< [in] volatility of underlying sigma
  const double T, ///< [in] time to maturity
  const MLIBInt optTypeInt ///< [in] option to compute
  const double& price ///< [out] price
)
{
  const mlib::finance::OptionType optType =
    static_cast<mlib::finance::OptionType >(optTypeInt);
  price =  mlib::finance::BSPrice(
             const double S, ///< [in] underlying
             const double K, ///< [in] strike
             const double r, ///< [in] risk-free rate
             const double v, ///< [in] volatility of underlying sigma
             const double T, ///< [in] time to maturity
             const OptionType optType ///< [in] option to compute
           );
}
