#include "../test.h"

#include "mlib/math/geometry/segment.h"

using namespace _mlib;

int main()
{
  print_title("Segment - 00");

  Point a(0,1);
  Point b(1,1);
  Segment ab(a,b);

  std::cout << ab.get_direction()
            << std::endl;
  std::cout << ab.get_extreme_points().first
            << std::endl;
  std::cout << ab.get_extreme_points().second
            << std::endl;

  Segment cd;
  cd = ab;
  std::cout << cd.get_direction()
            << std::endl;
  std::cout << cd.get_extreme_points().first
            << std::endl;
  std::cout << cd.get_extreme_points().second
            << std::endl;
}
