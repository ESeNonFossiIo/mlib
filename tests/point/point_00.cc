#include "../test.h"

#include "mlib/math/point.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title("Point");

  Point e1({1,0,0});
  Point e2({0,1,0});
  Point e3({1,1,1});

  std::cout << e1+e2;

  std::cout << e1;

  std::cout << e2;

  std::cout << (e1^e2);

  std::cout << (e1^e1);

  std::cout << (e2^e1);

  std::cout << (e1.t());

  std::cout << (e1.t()) *e1;

  std::cout << (e1) *e1.t();

  Point s;
  s.resize(3);
  s = e1 + e2;
  std::cout << s;

  Point e = e3;
  std::cout << e;
  Point ee = e2;
  std::cout << ee;
}
