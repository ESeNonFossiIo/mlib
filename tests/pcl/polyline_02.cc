#include "../test.h"

#include "mlib/pcl/polyline.h"

#include <iostream>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_PCL_WITH_VTK
  print_title("PCL polyline - 00");
  {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for(size_t i = 0; i < 5; ++i)
      {
        pcl::PointXYZI p;
        p.x = 0;
        p.y = 0;
        p.z = i;
        cloud->push_back(p);
      }

    for(size_t i = 4; i < 10; ++i)
      {
        pcl::PointXYZI p;
        p.x = 0;
        p.y = 0;
        p.z = i;
        cloud->push_back(p);
        cloud->push_back(p);
      }
    for(auto q: *cloud)
      std::cout << q << std::endl;
    std::cout << "------------------" << std::endl;
    remove_extra_points(cloud, 1);
    for(auto q: *cloud)
      std::cout << q << std::endl;

    std::cout << "------------------" << std::endl;
    add_missing_points(cloud, 0.25);
    remove_extra_points(cloud, 0.75);
    for(auto q: *cloud)
      std::cout << q << std::endl;

  }
#else //MLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/polyline_02");
#endif //MLIB_USE_PCL_WITH_VTK
}
