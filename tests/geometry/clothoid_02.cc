#include "../test.h"

#include "mlib/math/geometry/clothoid.h"

using namespace mlib;

int main()
{
  print_title("Clothoid");

  double clothoid_prm = 4.0;

  // Curvature:
  double k1           = -2.0;
  double k2           = 4.0;
  // ArcLength:
  double l1           = 0.0;
  double l2           = 20.0;

  double r    = std::sqrt((l2 - l1)/(M_PI * (k2 - k1)));


  // prm
  double t1           = k1 * r;
  double t2           = k2 * r;

  ApproximatedClothoid ap(r);

  double step = 0.01;

  std::cout << "============================================================="
            << std::endl
            << " k1 = " << k1
            << std::endl
            << " k2 = " << k2
            << std::endl
            << " l1 = " << l1
            << std::endl
            << " l2 = " << l2
            << std::endl
            << " t1 = " << t1
            << std::endl
            << " t2 = " << t2
            << std::endl
            << " r =  " << r
            << std::endl
            << "============================================================="
            << std::endl;

  double x = t1;
  while(x<=t2)
    {
      auto  p = ap(x);
      std::cout << x << " " << p.x() << " " << p.y() << " " << std::endl;
      x += step;
    }
}
