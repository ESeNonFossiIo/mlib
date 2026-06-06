#ifdef NUMERIX_USE_PCL

#ifndef _NUMERIX_PCL_CLUSTER_
#define _NUMERIX_PCL_CLUSTER_

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** \addtogroup PCL
 *  @{
 */

namespace numerix {

/**
 * @brief get_cluster
 * @param cloud
 * @param tolerance
 * @param min_pts_per_cluster
 * @return
 */
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
get_clusters(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
             const double& tolerance,
             const std::size_t& min_pts_per_cluster);

} // namespace numerix

/** @}*/

#endif //_NUMERIX_PCL_CLUSTER_

#endif // MYLIB_USE_PCL
