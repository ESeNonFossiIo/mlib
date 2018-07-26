#include "../test.h"

#include "mlib/math/arithmetic.h"

using namespace mlib;

int main()
{
  print_title("Arithmetic - min");

  std::vector<double> x = { 7, 5, 2, 5, 6, 1, 8, 12, 2, 3, 2 };
  std::cout << " argmin = " << argmin(x) << std::endl;
  std::cout << " min =    " << min(x) << std::endl;
}
