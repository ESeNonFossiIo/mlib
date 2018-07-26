#include "../test.h"
#include "../test_compare.h"

#include "mlib/pcl/segmentation.h"

#include <iostream>

using namespace _mlib;

int main()
{
#ifdef _MYLIB_USE_PCL_WITH_VTK
  print_title("PCL Best fitting plane");
  {
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new
                                                 pcl::PointCloud<pcl::PointXYZI>());

      double a = 1;
      double b = 2;
      double c = 3;
      double d = 4;
      double n = std::sqrt(a*a + b*b + c*c + d*d);
      a /= n;
      b /= n;
      c /= n;
      d /= n;

      for(unsigned int i = 0; i < 10; ++i)
        {
          for(unsigned int j = 0; j < 10; ++j)
            {
              for(unsigned int k = 0; k < 10; ++k)
                {
                  pcl::PointXYZI p;
                  p.x = i;
                  p.y = j;
                  p.z = (- a * p.x - b * p.y - d)/c;
                  p.intensity = 0;
                  cloud->push_back(p);
                }
            }
        }



      pcl::ModelCoefficients coef = fitting_plane(cloud);
      double nn = 0.0;
      for(size_t i = 0; i<4; i++)
        nn += coef.values[i]*coef.values[i];
      nn = std::sqrt(nn);

      std::cout <<  "error a -> "<< are_equal(a - coef.values[0]/nn,
                                              1e-6) << std::endl;
      std::cout <<  "error b -> "<< are_equal(b - coef.values[1]/nn,
                                              1e-6) << std::endl;
      std::cout <<  "error c -> "<< are_equal(c - coef.values[2]/nn,
                                              1e-6) << std::endl;
      std::cout <<  "error a -> "<< are_equal(d - coef.values[3]/nn,
                                              1e-6) << std::endl;

    }

  }
#else //_MYLIB_USE_PCL_WITH_VTK
  make_test_pass("pcl/best_fitting_plane_00");
#endif //_MYLIB_USE_PCL_WITH_VTK
}
