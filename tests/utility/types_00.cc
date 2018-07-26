#include "../test.h"

#include "mlib/utility/types.h"

#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Zeros");

  std::cout << " double        " << zero<double>() << std::endl;
  std::cout << " float         " << zero<float>() << std::endl;
  std::cout << " unsigned int  " << zero<unsigned int>() << std::endl;
  std::cout << " int           " << zero<int>() << std::endl;

  return 0;
}
