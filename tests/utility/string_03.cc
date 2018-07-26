#include "../test.h"

#include "mlib/utility/string.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title(" TEST for Utility - trim");

  std::string prova("abs \t n\n asdf \n  \n sdf  fasd ");

  std::cout <<  prova  << std::endl;
  std::cout <<  trim(prova)  << std::endl;

  return 0;
}
