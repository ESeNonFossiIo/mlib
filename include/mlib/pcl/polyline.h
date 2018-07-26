#ifdef _MYLIB_USE_PCL

#ifndef __MYLIB_PCL_POLYLINE_
#define __MYLIB_PCL_POLYLINE_

#include <mlib/math/angle.h>

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** \addtogroup PCL
 *  @{
 */

namespace _mlib
{

  void remove_angles(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                     const double& min_degree_angle);

  void add_missing_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                          const double& min_distance);

  void remove_extra_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                           const double& max_distance);

};
/** @}*/

#endif //__MYLIB_PCL_POLYLINE_

#endif //MYLIB_USE_PCL
