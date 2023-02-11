#include "../test.h"

#include <mlib/finance/black_scholes.h>

#include <iostream>

using namespace mlib;

int main()
{
  using namespace mlib::finance;

  print_title("Black-Scholes");

  // First we create the parameter list
  const double S = 100.0;  // Option price
  const double K = 100.0;  // Strike price
  const double r = 0.05;   // Risk-free rate (5%)
  const double v = 0.2;    // Volatility of the underlying (20%)
  const double T = 1.0;    // One year until expiry

  // Then we calculate the call/put values
  double call = BSPrice(
                  S, ///< [in] underlying
                  K, ///< [in] strike
                  r, ///< [in] risk-free rate
                  v, ///< [in] volatility of underlying sigma
                  T, ///< [in] time to maturity
                  OptionType::EuropeanCall ///< [in] option to compute
                );

  double put = BSPrice(
                 S, ///< [in] underlying
                 K, ///< [in] strike
                 r, ///< [in] risk-free rate
                 v, ///< [in] volatility of underlying sigma
                 T, ///< [in] time to maturity
                 OptionType::EuropeanPut ///< [in] option to compute
               );

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
