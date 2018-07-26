#include "../test.h"

#include "mlib/math/point.h"

#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Centroid");

  std::vector<Point>   p;
  std::vector<double>  w0 = {0.0, 0.1};
  std::vector<double>  w1 = {0.1, 0.0};
  std::vector<double>  w05 = {0.1, 0.1};

  Point e1(1, 0, 0);
  Point e2(0, 1, 0);
  p.push_back(e1);
  p.push_back(e2);

  std::cout << centroid(p, w0) << std::endl;
  std::cout << centroid(p, w1) << std::endl;
  std::cout << centroid(p, w05) << std::endl;
}
