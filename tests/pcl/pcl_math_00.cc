#include "../test.h"

#include "mlib/pcl/math.h"

#include <iostream>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_PCL
  print_title("PCL Math utility");

  pcl::PointXYZI p;
  pcl::PointXYZI q;

  p.x = 1.0;
  q.x = 2.0;

  std::cout << points_distance(p,q) << std::endl;
#else //MLIB_USE_PCL
  make_test_pass("pcl/pcl_math_00");
#endif //MLIB_USE_PCL
}
