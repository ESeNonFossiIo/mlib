#include "../test.h"

#include "mlib/utility/types.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title("Zeros");

  std::cout << " double        " << zero<double>() << std::endl;
  std::cout << " float         " << zero<float>() << std::endl;
  std::cout << " std::size_t  " << zero<std::size_t>() << std::endl;
  std::cout << " int           " << zero<int>() << std::endl;

  return 0;
}
