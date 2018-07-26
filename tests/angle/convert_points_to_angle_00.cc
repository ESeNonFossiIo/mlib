#include "../test.h"
#include "../test_compare.h"

#include "mlib/math/angle.h"

using namespace _mlib;

int main()
{
  print_title("Convert points to angle");

  for(unsigned int i = 0; i < 181; i += 5)
    {
      Angle alpha(i, AngleType::deg);

      Point a(1,0,0);
      Point b(std::cos(alpha.rad()),std::sin(alpha.rad()),0);
      Point o(0,0,0);

      Angle angle = get_angle_from_points(a,o,b);

      std::cout << "  ================================================= " <<
                std::endl;
      std::cout << "      i => " << are_equal(alpha.deg() - angle.deg()) << std::endl;
      std::cout << "           " << alpha.deg() << " vs " << angle.deg() << std::endl;
      std::cout << "  ================================================= " <<
                std::endl;
    }

  return 0;
}
