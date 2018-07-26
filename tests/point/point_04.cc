#include "../test.h"

#include "mlib/math/point.h"

#include <iostream>
#include <tuple>

using namespace _mlib;

int main()
{
  print_title("Point");

  // TODO: fix initializer_list list for 1 element
  /*
  {
    Point e1({1.0});
    Point e2({e1.x()});
    Point e3(e1.x());

    std::cout << e1 << std::endl;
    std::cout << e1.x() << std::endl;
    std::cout << e2 << std::endl;
    std::cout << e3 << std::endl;
  }
  */

  {
    Point e1(1.0);
    Point e2(e1);

    std::cout << e1 << std::endl;
    std::cout << e2 << std::endl;
  }

  {
    Point e1({1,0,0});
    std::cout << e1 << std::endl;
  }

  {
    Point e1(1,0,0);
    std::cout << e1 << std::endl;
  }

  {
    Point e1({1,0,0});
    Point e2(e1);

    std::cout << e1 << std::endl;
    std::cout << e2 << std::endl;
  }

  // {
  //   Point e1(1,0,0);
  //   Point e2(e1);
  //
  //   std::cout << e1 << std::endl;
  //   std::cout << e2 << std::endl;
  // }
}
