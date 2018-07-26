#include "../test.h"

#include "mlib/utility/color.h"

#include <iostream>

using namespace mlib;
using namespace mlib::Color;

int main()
{
  print_title("Color");

  std::cout << red.init()
            << " red "
            << red.end() << std::endl;
  std::cout << yellow.init()
            << " yellow "
            << yellow.end() << std::endl;
  std::cout << green.init()
            << " green "
            << green.end() << std::endl;
  std::cout << blue.init()
            << " blue "
            << blue.end() << std::endl;
  return 0;
}
