#include "mlib/math/euclidean_geometry.h"

#include <iostream>

using namespace _mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for middle_hyperplane_between_points" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  {
    Point e1({1,0});
    std::cout << e1;

    Point e2({2,0});
    std::cout << e2;

    HyperPlane  q = middle_hyperplane_between_points(e1,e2);
    std::cout << q;
  }

  {
    Point e1({1,0,0});
    std::cout << e1;

    Point e2({2,0,0});
    std::cout << e2;

    HyperPlane  q = middle_hyperplane_between_points(e1,e2);
    std::cout << q;
  }



}
