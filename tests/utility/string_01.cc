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

  std::string prova = from_double_to_str(3.13);
  std::cout <<  prova  << std::endl;

  return 0;
}
