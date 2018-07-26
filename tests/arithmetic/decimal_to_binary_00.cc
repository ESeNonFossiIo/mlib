#include "../test.h"

#include "mlib/math/arithmetic.h"
#include "mlib/utility/output.h"

using namespace _mlib;

int main()
{
  print_title("Arithmetic - decimal_to_binary");

  for(unsigned int i = 0; i < 15; ++i)
    {
      std::cout << " ========================================= "  << std::endl;
      std::cout << "     " << i << " = "<<decimal_to_binary(i) << std::endl;
      std::cout << "     " << i << " = "<<binary_to_decimal(decimal_to_binary(
                                                              i)) << std::endl;
      std::cout << " ========================================= "  << std::endl;
    }
}
