#ifdef MLIB_USE_PCL

#ifndef _MLIB_PCL_MATH_
#define _MLIB_PCL_MATH_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** \addtogroup PCL
 *  @{
 */

namespace mlib
{

  template<typename PointType>
  double
  scalar_product(
    const PointType& a,
    const PointType& b
  )
  {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  };

  /**
    * @brief distance
    * @param p1 first point
    * @param p2 second point
    */
  double
  points_distance(
    const pcl::PointXYZI& p1,
    const pcl::PointXYZI& p2);

  float
  norm(const pcl::PointXYZI& p);

  pcl::PointXYZI
  highest_point(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

  double
  min_distance(
    pcl::PointXYZI& p,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    double tolerance = 0.0);

  void
  get_closest_points_and_minimum_distance(
    pcl::PointXYZI& p,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    pcl::PointXYZI& closest_point,
    double& min_d);

  void
  neighborhood(
    pcl::PointCloud<pcl::PointXYZI>::Ptr input,
    pcl::PointXYZI& p,
    double& radius,
    pcl::PointCloud<pcl::PointXYZI>::Ptr neighborhood);


  /**
  * @brief get_farests_points
  * @param cloud
  * @param max_distance
  * @param i1
  * @param i2
  * @return
  */
  bool
  get_farests_points(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float& max_distance,
    size_t& i1,
    size_t& i2);

  /**
   * @brief get_closest_points
   * @param cloud1 first point cloud set
   * @param cloud2 second point cloud set
   * @param min_distance (calculated) the minimum distance between cloud1 and cloud2
   * @param i1_min (calculated) index of the argmin of the first cloud
   * @param i2_min (calculated) index of the argmin of the second cloud
   */
  bool
  get_closest_points(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2,
    float& min_distance,
    size_t& i1_min,
    size_t& i2_min);

};


/** @}*/

#endif //_MLIB_PCL_MATH_

#endif //MYLIB_USE_PCL
