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

// Test truncate function:
////////////////////////////////////////////////////////////////////////////////
  std::cout << " "
            << truncate<int> (98, 99, 100) << " / "
            << truncate<int> (98, 10, 97) << " / "
            << truncate<int> (98, 10, 100) << " "
            << std::endl;

  return 0;
}
