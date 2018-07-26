#include "../test.h"

#include "mlib/math/point.h"

#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Point");

  {
    Point e1({1,0,0});
    Point e2({1,1,0});

    std::cout << points_distance(e1,e2) << std::endl;
  }

  {
    Point e1({0,0,0});
    Point e2({1,1,0});

    std::cout << points_distance(e1,e2) << std::endl;
  }

  {
    Point e1({0,0,0});
    Point e2({1,1,0});

    std::cout << points_distance(e1,e2,1) << std::endl;
  }

  {
    Point e1({0,3,0});
    Point e2({4,0,0});

    std::cout << points_distance(e1,e2,2) << std::endl;
  }
}
