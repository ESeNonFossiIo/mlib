#include "mlib/math/euclidean_geometry.h"

#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for get_hyperplanes_intersection" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  {
    HyperPlane e1({1,-1,-1});
    std::cout << e1;

    HyperPlane e2({1,1,-7});
    std::cout << e2;

    Point  q = get_hyperplanes_intersection(e1,e2);
    std::cout << q;
  }
}
