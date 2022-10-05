#include "../test.h"

#include "mlib/math/geometry/clothoid.h"
#include "mlib/math/matrix/utility.h"

using namespace mlib;

int main()
{
  print_title("Clothoid");

  // Curvature:
  double k1           = 2.0;
  double k2           = 4.0;
  // ArcLength:
  double l1           = 0.0;
  double l2           = 1.0;

  // Clothoid prm:
  double r            = std::sqrt((l2 - l1)/(M_PI * (k2 - k1)));

  double theta        = M_PI / 3.0;
  double d            = 0.15;

  // Position extrem point
  Point p1(1.0,1.0);
  Point p2(d * std::cos(theta), d * std::sin(theta));
  p2 += p1;

  // prm clothoid
  double t1           = k1 * r;
  double t2           = k2 * r;

  ApproximatedClothoid ap(r);

  Point q1 = ap(t1);
  Point q2 = ap(t2);

  Point t;
  Matrixd R;
  rototranslation_matrix_2d(p1,p2,q1,q2,R,t);

  //
  // std::cout << R*(ap(t1) - q1) + q1 + t;
  // std::cout << p1;
  //
  // std::cout << R*(ap(t2) - q1) + q1 + t;
  // std::cout << p2;
  double step = 0.01;
  double x = t1;
  while(x<=t2)
    {
      auto  p = R*(ap(x)-q1) + q1 + t;
      std::cout << x << " " << p[0] << " " << p[1] << " " << std::endl;
      x += step;
    }
}
