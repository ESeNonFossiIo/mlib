#include "../test.h"

#include "mlib/utility/string.h"
#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - Upper/Lower case" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  std::string prova("SasgfRsdfSRsfdA");

  std::cout <<  prova  << std::endl;
  std::cout <<  lower_case(prova)  << std::endl;
  std::cout <<  upper_case(prova)  << std::endl;

  return 0;
}
