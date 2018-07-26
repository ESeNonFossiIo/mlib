#include "../test.h"

#include "mlib/pcl/math.h"

#include <iostream>

using namespace _mlib;

int main()
{
#ifdef _MYLIB_USE_PCL
  print_title("PCL Math utility");

  pcl::PointXYZI p;
  pcl::PointXYZI q;

  p.x = 1.0;
  q.x = 2.0;

  std::cout << points_distance(p,q) << std::endl;
#else //_MYLIB_USE_PCL
  make_test_pass("pcl/math_00");
#endif //_MYLIB_USE_PCL
}
