#include "mlib/math/utility.h"
#include <iostream>

// per std::setw
#include <iomanip>

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

// Test remove_singularities function:
////////////////////////////////////////////////////////////////////////////////
  auto f = normalize_range(1.0, 10.0);

  for(unsigned int i = 1.0; i <= 10; ++i)
    std::cout << f((double)i) << "\t ";

  return 0;
}
