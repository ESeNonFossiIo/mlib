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

// Test truncate_decimals function:
////////////////////////////////////////////////////////////////////////////////
  std::cout << " "
            << truncate_decimals(98.44,10) << " / "
            << truncate_decimals(98.44, 1) << " / "
            << truncate_decimals(98.44,.1) << " "
            << std::endl;

  return 0;
}
