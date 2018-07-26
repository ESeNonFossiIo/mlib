#include "../test.h"

#ifdef MLIB_USE_PCL_WITH_VTK
#include "mlib/pcl/polyline.h"
#include "mlib/pcl/io.h"
#endif //MLIB_USE_PCL_WITH_VTK

#include <iostream>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_PCL_WITH_VTK
  print_title("PCL polyline - 03");
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    load_PCD(get_test_dir()+"/pcl/poly_test.pcd", *cloud);

    remove_angles(cloud, 170);

    add_missing_points(cloud, 0.5);
    remove_extra_points(cloud, 0.75);

    save_txt(get_test_dir()+"/pcl/poly_test.txt", *cloud);
  }
#else //MLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/polyline_03");
#endif //MLIB_USE_PCL_WITH_VTK
}
