#include "../test.h"

#include "mlib/pcl/polyline.h"

#include <iostream>

using namespace _mlib;

int main()
{
#ifdef _MYLIB_USE_PCL_WITH_VTK
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

    pcl::PointXYZI p;
    p.x = 0;
    p.y = 1;
    p.z = 5;
    cloud->push_back(p);

    for(size_t i = 5; i < 10; ++i)
      {
        pcl::PointXYZI p;
        p.x = 0;
        p.y = 0;
        p.z = i;
        cloud->push_back(p);
      }
    for(auto q: *cloud)
      std::cout << q << std::endl;
    std::cout << "------------------" << std::endl;
    remove_angles(cloud, 160);
    for(auto q: *cloud)
      std::cout << q << std::endl;

  }
#else //_MYLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/polyline_00");
#endif //_MYLIB_USE_PCL_WITH_VTK
}
