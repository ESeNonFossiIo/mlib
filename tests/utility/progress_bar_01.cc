#include "mlib/utility/progress_bar.h"
#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

// Test ProgressBar class:
////////////////////////////////////////////////////////////////////////////////
  ProgressBar pb(20);

  pb(0);
  std::cout << std::endl;
  pb(20);
  std::cout << std::endl;
  pb(40);
  std::cout << std::endl;
  pb(60);
  std::cout << std::endl;
  pb(80);
  std::cout << std::endl;
  pb(100);
  std::cout << std::endl;

  return 0;

}
