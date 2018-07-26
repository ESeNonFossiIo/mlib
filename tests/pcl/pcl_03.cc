#include "../test.h"

#include "mlib/math/point.h"

#include <iostream>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_PCL_WITH_VTK
  print_title("PCL Conversion to Point");
  {

    pcl::PointXYZI p;
    p.x = 1;
    p.y = 2;
    p.z = 3;
    p.intensity = 0;

    Point q(p);
    std::cout << q;
  }
#else //MLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/pcl_03");
#endif //MLIB_USE_PCL_WITH_VTK
}
