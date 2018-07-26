#include "../test.h"

#include "mlib/math/utility.h"
#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Difference function - step version");

// Test difference function:
////////////////////////////////////////////////////////////////////////////////
  std::vector<double> v = {1,2,3,8,4,5,6,7,8,9,10};
  std::cout << " ";
  auto u = difference(v,true);
  auto w = difference(v,false);

  for(unsigned int i = 1; i < v.size(); ++i)
    std::cout << " " << u[i] << " - " << w[i] << std::endl;

  return 0;
}
