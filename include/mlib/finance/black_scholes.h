#ifdef MLIB_USE_EIGEN3

#ifndef _MLIB_EIGEN_CONVERSION_
#define _MLIB_EIGEN_CONVERSION_

#include "mlib/math/matrix/matrix.h"

#include <iostream>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

/** \addtogroup math
 *  @{
 */

namespace mlib
{

  // This calculates d_j, for j in {1,2}. This term appears in the closed
  // form solution for the European call or put price
  double d_1(const double S, const double K, const double r,
             const double v, const double T)
  {
    return (log(S/K) + (r + 0.5*v*v)*T)/(v*(sqrt(T)));
  }
  // This calculates d_j, for j in {1,2}. This term appears in the closed
  // form solution for the European call or put price
  double d_2(const double S, const double K, const double r,
             const double v, const double T)
  {
    return (log(S/K) + (r - 0.5*v*v)*T)/(v*(sqrt(T)));
  }

  // Calculate the European vanilla call price based on
  // underlying S, strike K, risk-free rate r, volatility of
  // underlying sigma and time to maturity T
  double BSPrice(
    const double S, ///< [in] underlying
    const double K, ///< [in] strike
    const double r, ///< [in] risk-free rate
    const double v, ///< [in] volatility of underlying sigma
    const double T ///< [in] time to maturity
  )
  {
    if(option == OptionType::Call)
      {
        return S * norm_cdf(d_1(S, K, r, v, T))-K*exp(-r*T) * norm_cdf(d_2(S, K,
                                                                           r, v, T));
      }
    else
      {
        return -S*norm_cdf(-d_1(S, K, r, v, T))+K*exp(-r*T) * norm_cdf(-d_2(S, K,
                                                                            r, v, T));
      }
  }


  int main(int argc, char **argv)
  {
    // First we create the parameter list
    double S = 100.0;  // Option price
    double K = 100.0;  // Strike price
    double r = 0.05;   // Risk-free rate (5%)
    double v = 0.2;    // Volatility of the underlying (20%)
    double T = 1.0;    // One year until expiry

    // Then we calculate the call/put values
    double call = call_price(S, K, r, v, T);
    double put = put_price(S, K, r, v, T);

    // Finally we output the parameters and prices
    std::cout << "Underlying:      " << S << std::endl;
    std::cout << "Strike:          " << K << std::endl;
    std::cout << "Risk-Free Rate:  " << r << std::endl;
    std::cout << "Volatility:      " << v << std::endl;
    std::cout << "Maturity:        " << T << std::endl;

    std::cout << "Call Price:      " << call << std::endl;
    std::cout << "Put Price:       " << put << std::endl;

    return 0;
  }

};


/** @}*/

#endif //_MLIB_EIGEN_CONVERSION_

#endif //MLIB_USE_EIGEN3
