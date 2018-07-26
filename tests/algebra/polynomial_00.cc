#include "mlib/math/algebra.h"

#include <iostream>
#include <vector>

using namespace _mlib;

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
    std::cout << "c   = " << p << std::endl;
    std::cout << "deg = " << p.deg() << std::endl;
    std::cout << "c   = " << p.d(1) << std::endl;
    std::cout << "deg = " << p.d(1).deg() << std::endl;
    std::cout << "c   = " << p.d(2) << std::endl;
    std::cout << "deg = " << p.d(2).deg() << std::endl;
    std::cout << "c   = " << p.d(3) << std::endl;
    std::cout << "deg = " << p.d(3).deg() << std::endl;
  }
  std::cout <<
            "=================================================" <<
            std::endl;
  {
    Polynomial q({1,0,3,4});
    std::cout << "c   = " << q << std::endl;
    std::cout << "deg = " << q.deg() << std::endl;
    std::cout << "c   = " << q.d(1) << std::endl;
    std::cout << "deg = " << q.d(1).deg() << std::endl;
    std::cout << "c   = " << q.d(2) << std::endl;
    std::cout << "deg = " << q.d(2).deg() << std::endl;
    std::cout << "c   = " << q.d(3) << std::endl;
    std::cout << "deg = " << q.d(3).deg() << std::endl;
  }
  std::cout <<
            "=================================================" <<
            std::endl;
}
