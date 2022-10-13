#include "../test.h"

#include <mlib/finance/black_scholes.h>

#include <iostream>

using namespace mlib;

int main()
{
  print_title("Black-Scholes");

  std::vector<double> v = {1, 2, 3, 4, 5, 6};

  std::cout << " mean      = " << mean(v) << std::endl;
  std::cout << " var       = " << var(v) << std::endl;
  std::cout << " stddev    = " << stddev(v) << std::endl;

  return 0;
}
