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

// Test accumulate function:
////////////////////////////////////////////////////////////////////////////////
  std::vector<double> v = {1,2,3,4,5,6,7,8,9,10};
  std::cout << " ";
  auto u = accumulate(v,true);
  auto w = accumulate(v,false);

  for(unsigned int i = 1; i < v.size(); ++i)
    std::cout << " " << u[i] << " - " << w[i] << std::endl;

  return 0;
}
