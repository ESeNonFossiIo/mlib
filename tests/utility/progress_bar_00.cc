#include "mlib/utility/progress_bar.h"
#include <iostream>

using namespace _mlib;

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

  pb.print_bar(0);
  std::cout << std::endl;
  pb.print_bar(20);
  std::cout << std::endl;
  pb.print_bar(40);
  std::cout << std::endl;
  pb.print_bar(60);
  std::cout << std::endl;
  pb.print_bar(80);
  std::cout << std::endl;
  pb.print_bar(100);
  std::cout << std::endl;

  return 0;
}
