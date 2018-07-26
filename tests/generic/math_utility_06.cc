#include "mlib/math/utility.h"
#include <iostream>

using namespace _mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Math Utility" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

// Test straight_part function:
////////////////////////////////////////////////////////////////////////////////
  std::vector<double> v = {1.01, 1.2, 1.21, 1.22, 1.23, 1, 3, 5, 5.1, 5.11};
  auto u = straight_part(v, 0.1);

  for(unsigned int i = 0; i < u.size(); ++i)
    std::cout << " " << u[i] << std::endl;

  return 0;

}
