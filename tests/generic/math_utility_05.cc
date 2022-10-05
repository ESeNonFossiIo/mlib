#include "mlib/math/utility.h"
#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Math Utility" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

// Test truncate_decimals_vec function:
////////////////////////////////////////////////////////////////////////////////
  std::vector<double> v = {1.123,2.123,3.132,4.312,5.312,6.31,7.31,8.31};
  std::cout << " ";
  auto u = truncate_decimals_vec(v,0.1);
  auto w = truncate_decimals_vec(v,0.01);

  for(std::size_t i = 0; i < v.size(); ++i)
    std::cout << " " << v[i] << " - "<< u[i] << " - " << w[i] <<
              std::endl;

  return 0;
}
