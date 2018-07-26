#include "../test.h"

#include "mlib/utility/string.h"
#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - Conversion from string" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  std::cout << from_str_to_double("3.13") << std::endl;
  std::cout << from_str_to_unsigned_int("3") << std::endl;
  std::cout << from_str_to_int("-3") << std::endl;
  std::cout << from_str_to_double("True") << std::endl;
  return 0;
}
