#include "../test.h"

#include "mlib/math/utility.h"
#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Difference function - xy version");

// Test difference function:
////////////////////////////////////////////////////////////////////////////////
  std::vector<double> y = {1,2,3,4,5,6,7,8,9,10,11};
  std::vector<double> x1 = {2,4,6,8,10,12,14,16,18,20,22};
  std::vector<double> x2 = {1,2,3,4,5,6,7,8,9,10,11};
  std::cout << " ";
  auto u = difference(y, x1);
  auto w = difference(y, x2);

  for(unsigned int i = 1; i < y.size(); ++i)
    std::cout << " " << u[i] << " - " << w[i] << std::endl;

  return 0;
}
