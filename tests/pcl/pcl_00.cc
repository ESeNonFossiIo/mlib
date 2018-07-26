#include "../test.h"

#include "mlib/pcl/io.h"

#include <iostream>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_PCL_WITH_VTK
  print_title("PCL Wrapper");
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.width    = 5;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for(size_t i = 0; i < cloud.points.size(); ++i)
      {
        cloud.points[i].x = i + 0;
        cloud.points[i].y = i + 1;
        cloud.points[i].z = i + 2;
      }

    save_PCD("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd."
              << std::endl;
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(load_PCD("test_pcd.pcd", *cloud) == -1)  //* load the file
      {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
      }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for(size_t i = 0; i < cloud->points.size(); ++i)
      std::cout << "    " << cloud->points[i].x
                << " "    << cloud->points[i].y
                << " "    << cloud->points[i].z << std::endl;

  }

#else //MLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/pcl_00");
#endif //MLIB_USE_PCL_WITH_VTK
}
