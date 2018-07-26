#ifdef MLIB_USE_PCL

#include "mlib/pcl/math.h"

namespace mlib
{
  double points_distance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2)
  {
    Eigen::Vector3f diff = Eigen::Vector3f(p1.x - p2.x,
                                           p1.y - p2.y,
                                           p1.z - p2.z);
    return diff.norm();
  }

  float norm(const pcl::PointXYZI& p)
  {
    Eigen::Vector3f diff = Eigen::Vector3f(p.x,
                                           p.y,
                                           p.z);
    return (diff.norm());
  }


  pcl::PointXYZI
  highest_point(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    pcl::PointXYZI max;
    max.z = std::numeric_limits<float>::lowest();
    for(auto q: *cloud)
      {
        if(max.z < q.z)
          {
            max = q;
          }
      }
    return max;
  }

  double
  min_distance(
    pcl::PointXYZI& p,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    double tolerance)
  {
    double min = std::numeric_limits<double>::max();
    for(auto q: *cloud)
      {
        double d = points_distance(p,q);
        if(d < tolerance)
          {
            return 0.0;
          }
        else if(d<min)
          {
            min = d;
          }
      }
    return min;
  }

  void
  get_closest_points_and_minimum_distance(
    pcl::PointXYZI& p,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    pcl::PointXYZI& closest_point,
    double& min_d)
  {
    assert(cloud->size() > 0);
    min_d = std::numeric_limits<double>::max();
    for(auto q: *cloud)
      {
        if(points_distance(p,q) < min_d)
          {
            min_d = points_distance(p,q);
            closest_point = q;
          }
      }
  }

  void
  neighborhood(
    pcl::PointCloud<pcl::PointXYZI>::Ptr input,
    pcl::PointXYZI& p,
    double& radius,
    pcl::PointCloud<pcl::PointXYZI>::Ptr neighborhood
  )
  {
    for(auto q: *input)
      {
        if(points_distance(p,q)<radius)
          {
            neighborhood->push_back(q);
          }
      }
  }

  bool
  get_farests_points(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float& max_distance,
    size_t& i1,
    size_t& i2)
  {
    // Initialize values:
    max_distance = std::numeric_limits<float>::lowest();
    i2 = 0;
    i2 = 0;
    bool status = false;

    for(size_t i = 0; i < cloud->points.size(); ++i)
      {
        for(size_t j = 0; j < cloud->points.size(); ++j)
          {
            float d = points_distance(cloud->points[i],cloud->points[j]);
            if(d>max_distance)
              {
                max_distance = d;
                i1 = i;
                i2 = j;
                status = true;
              }
          }
      }
    return status;
  }

  bool
  get_closest_points(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2,
    float& min_distance,
    size_t& i1_min,
    size_t& i2_min)
  {
    // Initialize values:
    min_distance = std::numeric_limits<float>::max();
    i2_min = 0;
    i2_min = 0;
    bool status = false;

    for(size_t i = 0; i < cloud1->points.size(); ++i)
      {
        for(size_t j = 0; j < cloud2->points.size(); ++j)
          {
            float d = points_distance(cloud1->points[i],cloud2->points[j]);
            if(d<min_distance)
              {
                min_distance = d;
                i1_min = i;
                i2_min = j;
                status = true;
              }
          }
      }
    return status;
  }

}

#endif // MLIB_USE_PCL
