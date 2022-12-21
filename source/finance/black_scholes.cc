#include <mlib/finance/black_scholes.h>
#include <mlib/math/statistics/norm_dist.h>

#include <cmath>

namespace
{
/// ----------------------------------------------------------------------------
  // This calculates d_j, for j in {1,2}. This term appears in the closed
  // form solution for the European call or put price
  double d_1(const double S, const double K, const double r,
             const double v, const double T)
  {
    return (std::log(S/K) + (r + 0.5*v*v)*T)/(v*(std::sqrt(T)));
  }

  /// ----------------------------------------------------------------------------
  // This calculates d_j, for j in {1,2}. This term appears in the closed
  // form solution for the European call or put price
  double d_2(const double S, const double K, const double r,
             const double v, const double T)
  {
    return (std::log(S/K) + (r - 0.5*v*v)*T)/(v*(std::sqrt(T)));
  }

}

namespace mlib
{

  namespace finance
  {

/// ----------------------------------------------------------------------------
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
    )
    {
      using namespace mlib::math::statistics;

      if(optType == OptionType::EuropeanCall)
        {
          return S * norm_cdf(d_1(S, K, r, v, T))-K*exp(-r*T) * norm_cdf(d_2(S, K,
                                                                             r, v, T));
        }
      else if(optType == OptionType::EuropeanPut)
        {
          return -S*norm_cdf(-d_1(S, K, r, v, T))+K*exp(-r*T) * norm_cdf(-d_2(S, K,
                                                                              r, v, T));
        }
      else
        {
          /// TODO: throw Unrecognised type
          return -1;
        }
    }

  }

}
