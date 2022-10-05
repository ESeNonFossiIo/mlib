#include "mlib/math/utility.h"
#include <iostream>

// per std::setw
#include <iomanip>

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

// Test remove_singularities function:
////////////////////////////////////////////////////////////////////////////////
  auto f = normalize_range(1.0, 10.0);

  for(std::size_t i = 1; i <= 10; ++i)
  {
    const double x = static_cast<double>(i);
    std::cout << f(x) << "\t ";
  }


  return 0;
}
