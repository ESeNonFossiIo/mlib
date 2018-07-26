#include "../test.h"

#include "mlib/math/geometry/segment.h"

using namespace mlib;

int main()
{
  print_title("Segment - 01");

  Point a(0,0);
  Point b(0,1);
  Segment ab(a,b);

  Point c(0,0);
  Point d(1,0);
  Segment cd(c,d);

  std::cout << " angle          = " << get_angle(ab, cd) << std::endl;
  std::cout << " closest points = "
            << closest_points(ab, cd).first
            << closest_points(ab, cd).second;


}
