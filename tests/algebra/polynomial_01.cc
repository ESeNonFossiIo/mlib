#include "mlib/math/algebra.h"

#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Polynomial" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;
  {
    Polynomial p({1,2,1});
    for(std::size_t i = 0; i <= 10; i++)
      std::cout << (double)i / 10.0 << " -> " << p((double)i / 10.0) << std::endl;
  }
  std::cout <<
            "=================================================" <<
            std::endl;
  {
    Polynomial p({1,0,3,4});
    for(std::size_t i = 0; i <= 10; i++)
      std::cout << (double)i / 10.0 << " -> " << p((double)i / 10.0) << std::endl;
  }
  std::cout <<
            "=================================================" <<
            std::endl;
}
