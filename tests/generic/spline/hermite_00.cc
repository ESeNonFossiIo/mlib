#include "../../test_compare.h"
#include "../../test.h"

#include "mlib/math/point.h"
#include "mlib/math/spline.h"

#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Hermite Spline");

  Point p1(0,1,0);
  Point p2(1,0,0);
  Point v1(0,1,0);
  Point v2(1,0,0);

  double t1(3.0);
  double t2(5.0);

  HermiteSpline<3> hs(p1,p2,v1,v2,t1,t2);

  std::cout << (are_equal(hs.p(t1), p1) ? "[OK]" : "[Fail]") << std::endl;
  std::cout << (are_equal(hs.p(t2), p2) ? "[OK]" : "[Fail]") << std::endl;

  std::cout << (are_equal(hs.v(t1), v1) ? "[OK]" : "[Fail]") << std::endl;
  std::cout << (are_equal(hs.v(t2), v2) ? "[OK]" : "[Fail]") << std::endl;

  return 0;
}
