#include "../test.h"

#ifdef NUMERIX_USE_PCL_WITH_VTK
#include "numerix/pcl/io.h"
#include "numerix/pcl/polyline.h"
#endif // NUMERIX_USE_PCL_WITH_VTK

#include <iostream>

using namespace numerix;

int main()
{
#ifdef NUMERIX_USE_PCL_WITH_VTK
    print_title("PCL polyline - 03");
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        load_PCD(get_test_dir() + "/pcl/poly_test.pcd", *cloud);

        remove_angles(cloud, 170);

        add_missing_points(cloud, 0.5);
        remove_extra_points(cloud, 0.75);

        save_txt(get_test_dir() + "/pcl/poly_test.txt", *cloud);
    }
#else  // NUMERIX_USE_PCL_WITH_VTK
    make_test_pass("pcl/polyline_03");
#endif // NUMERIX_USE_PCL_WITH_VTK
}
