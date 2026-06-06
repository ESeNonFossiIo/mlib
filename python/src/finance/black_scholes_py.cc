#include <numerix/core/export.h>
#include <numerix/finance/black_scholes.h>

#include "_python/status.h"

/// ----------------------------------------------------------------------------
// Calculate the European vanilla call price based on
// underlying S, strike K, risk-free rate r, volatility of
// underlying sigma and time to maturity T
NUMERIX_EXPORT NUMERIXStatus NUMERIX_BSPricer(const double S, ///< [in] underlying
                                     const double K, ///< [in] strike
                                     const double r, ///< [in] risk-free rate
                                     const double v, ///< [in] volatility of underlying sigma
                                     const double T, ///< [in] time to maturity
                                     const NUMERIXInt optTypeInt, ///< [in] option to compute
                                     double* price             ///< [out] price
)
{
    const numerix::finance::OptionType optType = static_cast<numerix::finance::OptionType>(optTypeInt);
    *price = numerix::finance::BSPrice(S, K, r, v, T, optType);
    return NUMERIXStatus::Success;
}
