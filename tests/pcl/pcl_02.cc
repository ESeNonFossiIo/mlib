#include "../test.h"

#include "mlib/pcl/bounding_box.h"

#include <iostream>

using namespace _mlib;

int main()
{
#ifdef _MYLIB_USE_PCL_WITH_VTK
  print_title("PCL BoundingBox");
  {
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new
                                                 pcl::PointCloud<pcl::PointXYZI>());

      for(unsigned int i = 0; i < 2; ++i)
        {
          for(unsigned int j = 0; j < 2; ++j)
            {
              for(unsigned int k = 0; k < 2; ++k)
                {
                  pcl::PointXYZI p;
                  p.x = i;
                  p.y = j;
                  p.z = k/4.0;
                  p.intensity = 0;
                  cloud->push_back(p);
                }
            }
        }

      BoundingBox bb(cloud);

      std::cout << " is flat   = "
                << bb.is_flat()
                << std::endl;

      std::cout <<  " tolerance = "
                << 0.3
                << std::endl;
      std::cout << " is flat   = "
                << bb.is_flat(0.3)
                << std::endl;
    }

    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new
                                                 pcl::PointCloud<pcl::PointXYZI>());

      for(unsigned int i = 0; i < 2; ++i)
        {
          for(unsigned int j = 0; j < 2; ++j)
            {
              for(unsigned int k = 0; k < 2; ++k)
                {
                  pcl::PointXYZI p;
                  p.x = i;
                  p.y = j;
                  p.z = 0;
                  p.intensity = 0;
                  cloud->push_back(p);
                }
            }
        }

      BoundingBox bb(cloud);

      std::cout << " is flat   = "
                << bb.is_flat()
                << std::endl;
    }
  }
#else //_MYLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/pcl_02");
#endif //_MYLIB_USE_PCL_WITH_VTK
}
