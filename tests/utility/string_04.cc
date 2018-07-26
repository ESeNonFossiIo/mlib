#include "../test.h"

#include "mlib/utility/string.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title(" TEST for Utility - trim");

  auto v = split("1;;2",";",true);

  for(auto p : v)
    {
      std::cout << " -> " << p << std::endl;
    }

  return 0;
}
