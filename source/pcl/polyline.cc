#ifdef MLIB_USE_PCL

#include "mlib/pcl/polyline.h"

namespace mlib
{
  void remove_angles(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                     const double& min_degree_angle)
  {
    auto it = input_cloud->begin();

    while(it != input_cloud->end())
      {

        auto nx  = std::next(it, 1);
        if(nx == input_cloud->end())
          break;

        auto nnx = std::next(it, 2);
        if(nnx == input_cloud->end())
          break;

        if(get_angle_from_points(*it, *nx, *nnx).deg() < min_degree_angle)
          {
            input_cloud->erase(nnx);
          }
        else
          {
            ++it;
          }

      };
  }

  void add_missing_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                          const double& min_distance)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_points(new
                                                    pcl::PointCloud<pcl::PointXYZI>);
    auto it = input_cloud->begin();
    while(it != input_cloud->end())
      {
        new_points->push_back(*it);

        auto nx  = std::next(it, 1);
        if(nx == input_cloud->end())
          break;

        if(points_distance(*it, *nx) > min_distance)
          {
            double ratio = points_distance(*it, *nx)/min_distance + 1.0;
            for(size_t n = 1; n<=ratio; ++n)
              {
                pcl::PointXYZI p;
                p.x = it->x + n * (nx->x - it->x)/ratio;
                p.y = it->y + n * (nx->y - it->y)/ratio;
                p.z = it->z + n * (nx->z - it->z)/ratio;
                p.intensity = it->intensity + n * (nx->intensity - it->intensity)/ratio;
                new_points->push_back(p);
              }
          }
        ++it;
      };
    *input_cloud = *new_points;
  }

  void remove_extra_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                           const double& max_distance)
  {

    auto it = input_cloud->begin();

    while(it != input_cloud->end())
      {

        auto nx  = std::next(it, 1);

        if(nx == input_cloud->end())
          break;

        if(points_distance(*it, *nx) < max_distance)
          {
            input_cloud->erase(nx);
          }
        else
          {
            ++it;
          }
      };
  }


}

#endif // MLIB_USE_PCL
