#ifdef NUMERIX_USE_PCL

#ifndef _NUMERIX_PCL_POLYLINE_
#define _NUMERIX_PCL_POLYLINE_

#include <numerix/math/angle.h>

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** \addtogroup PCL
 *  @{
 */

namespace numerix {

void remove_angles(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                   const double& min_degree_angle);

void add_missing_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                        const double& min_distance);

void remove_extra_points(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                         const double& max_distance);

} // namespace numerix
/** @}*/

#endif //_NUMERIX_PCL_POLYLINE_

#endif // MYLIB_USE_PCL
