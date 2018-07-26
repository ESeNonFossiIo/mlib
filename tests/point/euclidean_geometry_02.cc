#include "mlib/math/euclidean_geometry.h"

#include <iostream>

using namespace _mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for circumference_center" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  {
    Point e1({1,0});
    std::cout << e1;

    Point e2({1,1});
    std::cout << e2;

    Point e3({0,1});
    std::cout << e3;

    Point  q = circumference_center(e1,e2,e3);
    std::cout << q;
  }

  {
    Point e1({1,0,1});
    std::cout << e1;

    Point e2({1,1,1});
    std::cout << e2;

    Point e3({0,1,1});
    std::cout << e3;

    Point  q = circumference_center(e1,e2,e3);
    std::cout << q;
  }

}
