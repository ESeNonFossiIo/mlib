#include "../test.h"

#include "mlib/math/geometry/clothoid.h"

using namespace mlib;

int main()
{
  print_title("Clothoid");

  ApproximatedClothoid ap(1);

  double step = 0.01;
  double L    = 10.0;

  double x = 0.0;
  while(x<L)
    {
      auto  p = ap(x);
      std::cout << x << " " << p.x() << " " << p.y() << " " << std::endl;
      x += step;
    }
}
