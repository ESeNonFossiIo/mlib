#ifdef MLIB_USE_PCL

#ifndef _MLIB_PCL_CLUSTER_
#define _MLIB_PCL_CLUSTER_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** \addtogroup PCL
 *  @{
 */

namespace mlib
{

  /**
   * @brief get_cluster
   * @param cloud
   * @param tolerance
   * @param min_pts_per_cluster
   * @return
   */
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
  get_clusters(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const double& tolerance,
    const unsigned int& min_pts_per_cluster);

};


/** @}*/

#endif //_MLIB_PCL_CLUSTER_

#endif //MYLIB_USE_PCL
