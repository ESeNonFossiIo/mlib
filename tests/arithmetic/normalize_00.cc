#include "../test.h"

#include "mlib/math/arithmetic.h"
#include "mlib/utility/output.h"

using namespace mlib;

int main()
{
  print_title("Arithmetic - normalize");

  std::vector<double> x = { 7, 5, 2, 5, 6, 1, 8, 12, 2, 3, 2 };
  std::vector<double> y = normalize(x);
  std::cout << x << std::endl;
  std::cout << y << std::endl;
}
