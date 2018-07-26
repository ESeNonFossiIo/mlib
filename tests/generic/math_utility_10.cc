#include "../test.h"

#include "mlib/math/utility.h"
#include "mlib/utility/output.h"


using namespace _mlib;

int main()
{
  print_title("Force mean");

// Test difference function:
////////////////////////////////////////////////////////////////////////////////
  {
    std::vector<double> y = {1,2,3,7,8,9,1,2,3};
    std::vector<double> x = force_mean(y, 1, 1.2);

    std::cout << y << std::endl
              << x << std::endl ;
  }
  {
    std::vector<double> y = {1,3,2,3,1,7,9,8,9,7,1,3,2,3,1};
    std::vector<double> x = force_mean(y, 2, 1.2);

    std::cout << y << std::endl
              << x << std::endl ;
  }
  {
    std::vector<double> y = {1,3,2,3,1,1,3,2,3,1,7,9,8,9,7,7,9,8,9,7,7,9,8,9,7,7,9,8,9,7,1,3,2,3,1,1,3,2,3,1,1,3,2,3,1};
    std::vector<double> x = force_mean(y, 2, 1.2);

    std::cout << y << std::endl
              << x << std::endl ;
  }
  return 0;
}
