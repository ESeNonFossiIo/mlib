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

// Test sgn function:
////////////////////////////////////////////////////////////////////////////////
  std::cout << " "
            << sgn<int> (-1) << " / "
            << sgn<int> (1) << " / "
            << sgn<int> (0) << "  "
            << std::endl;

  std::cout << " "
            << sgn<double> (-1) << " / "
            << sgn<double> (1) << " / "
            << sgn<double> (0) << "  "
            << std::endl;

  return 0;
}
