#include "../test.h"

#include "mlib/pcl/bounding_box.h"

#include <iostream>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_PCL_WITH_VTK
  print_title("PCL BoundingBox - ged_dominant_direction");
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
                  p.z = k * 4.0;
                  p.intensity = 0;
                  cloud->push_back(p);
                }
            }
        }

      BoundingBox bb(cloud);

      Segment s = bb.get_dominant_direction();

      std::cout << s.get_extreme_points().first
                << s.get_extreme_points().second
                << s.get_direction()
                << std::endl;

      auto sizes = bb.sizes();
      for(auto size: sizes)
        {
          std::cout << " distance = "  << size << std::endl;
        }
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
                  p.y = j * 4.0;
                  p.z = k;
                  p.intensity = 0;
                  cloud->push_back(p);
                }
            }
        }

      BoundingBox bb(cloud);

      Segment s = bb.get_dominant_direction();

      std::cout << s.get_extreme_points().first
                << s.get_extreme_points().second
                << s.get_direction()
                << std::endl;

      auto sizes = bb.sizes();
      for(auto size: sizes)
        {
          std::cout << " distance = "  << size << std::endl;
        }
    }
  }
#else //MLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/pcl_bb_00");
#endif //MLIB_USE_PCL_WITH_VTK
}
