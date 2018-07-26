#include "../test.h"

#include "mlib/pcl/io.h"

#include <iostream>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_PCL_WITH_VTK
  print_title("PCL Wrapper");
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;

    cloud.width    = 100;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for(size_t i = 0; i < cloud.points.size(); ++i)
      {
        cloud.points[i].x = i + 0;
        cloud.points[i].y = i + 1;
        cloud.points[i].z = i + 2;
        cloud.points[i].intensity = i + 3;
      }

    save_txt("test_pcd.txt", cloud);

    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.txt."
              << std::endl;
  }
#else //MLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/pcl_01");
#endif //MLIB_USE_PCL_WITH_VTK
}
