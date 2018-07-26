#include "../test.h"

#include "mlib/math/arithmetic.h"

using namespace _mlib;

int main()
{
  print_title("Arithmetic - max");

  std::vector<double> x = { 7, 5, 2, 5, 6, 1, 8, 12, 2, 3, 2 };
  std::cout << " argmax = " << argmax(x) << std::endl;
  std::cout << " max =    " << max(x) << std::endl;
}
