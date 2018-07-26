#include "../test.h"

#include "mlib/math/point.h"

#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Point");

  {
    Point e1({1,0,0});
    std::cout << e1;
    e1.normalize();
    std::cout << e1;
  }

  {
    Point e1({1,1,0});
    std::cout << e1;
    e1.normalize();
    std::cout << e1;
  }

  {
    Point e1({1,1,1});
    std::cout << e1;
    e1.normalize();
    std::cout << e1;
  }
}
