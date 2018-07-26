#include "../test.h"

#include "mlib/math/geometry/clothoid.h"

#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Clothoid");

  Clothoid clothoid(1.0);

  double step = 0.01;
  double L    = 10.0;

  double x = 0.0;
  while(x<L)
    {
      auto  p = clothoid(x);
      std::cout << x << " " << p.x() << " " << p.y() << " " << std::endl;
      x += step;
    }
}
