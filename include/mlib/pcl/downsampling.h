#ifdef MLIB_USE_PCL

#ifndef _MLIB_PCL_DOWNSAMPLING_
#define _MLIB_PCL_DOWNSAMPLING_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
/** \addtogroup PCL
 *  @{
 */

namespace mlib
{

  /**
   * @brief voxel_reduction
   * @param cloud
   * @param downsampled
   * @param leaf
   */
  void
  voxel_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                     float cube_size);

  /**
   * @brief intensity_downsampling
   * @param cloud
   * @param downsampled
   * @param min
   * @param max
   */
  void
  intensity_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                         float min, float max);

  /**
   * @brief neighborhood_downsampling
   * @param cloud
   * @param downsampled
   * @param radius
   * @param num_points
   */
  void
  neighborhood_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                            float radius, unsigned int num_points);

  void
  horizontal_cloud_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                                float height, unsigned int num_points);

  void
  horizontal_section_cloud_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr
                                        cloud,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                                        float height, float thickness);

};


/** @}*/

#endif //_MLIB_PCL_DOWNSAMPLING_

#endif //MYLIB_USE_PCL
