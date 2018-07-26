#include "../test.h"

#include "mlib/math/arithmetic.h"
#include "mlib/utility/output.h"

using namespace _mlib;

int main()
{
  print_title("Arithmetic - Pythagoras");

  std::cout <<  " sqrt(9 + 16) = 5 = "<< pythagoras(3, 4) << std::endl;
}
