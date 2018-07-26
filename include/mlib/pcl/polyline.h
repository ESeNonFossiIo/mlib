#ifdef MLIB_USE_PCL

#ifndef _MLIB_PCL_POLYLINE_
#define _MLIB_PCL_POLYLINE_

#include <mlib/math/angle.h>

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** \addtogroup PCL
 *  @{
 */

namespace mlib
{

  void remove_angles(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                     const double& min_degree_angle);

  void add_missing_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                          const double& min_distance);

  void remove_extra_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                           const double& max_distance);

};
/** @}*/

#endif //_MLIB_PCL_POLYLINE_

#endif //MYLIB_USE_PCL
