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
  std::vector<double> v =
  {
    0, 0, 0, 0.00288806, 0.00647603, 0.00647603, 0.00647603, 0.00647603,
    0.00647603, 0.00647603, 0.00647603, 0.00647603, 0.00647603, 0.00647603,
    0.00647603, 0.00647603, 0.00647603, 0.00647603, 0.00647603, 0.00647603,
    0.00647603, 0.00647603, 0.00647603, 0.00647603, 0.00647603, 0.00647603,
    0.00647603, 0.00647603, 0.00980336, 0, 0, 0,
    0
  };

  auto u = remove_singularities(v);

  for(std::size_t i = 0; i < u.size(); ++i)
    std::cout << std::setw(8) << v[i] << "\t "
              << std::setw(8) << u[i] << std::endl;

  return 0;
}
