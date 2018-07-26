#include "../test.h"

#include "mlib/math/point.h"

#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Point");

  Point e1({1,0,0});
  Point e2 = -1*e1;

  std::cout << e1+e2;

  std::cout << e1;

  std::cout << e2;

  std::cout << (e1.t()) *e1;

  std::cout << (e1) *e1.t();

  Point s = e1 + e2;
  std::cout << s;

}
