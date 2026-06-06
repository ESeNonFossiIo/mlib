#ifdef NUMERIX_USE_PCL

#ifndef _NUMERIX_PCL_DOWNSAMPLING_
#define _NUMERIX_PCL_DOWNSAMPLING_

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
/** \addtogroup PCL
 *  @{
 */

namespace numerix {

/**
 * @brief voxel_reduction
 * @param cloud
 * @param downsampled
 * @param leaf
 */
void voxel_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                        float cube_size);

/**
 * @brief intensity_downsampling
 * @param cloud
 * @param downsampled
 * @param min
 * @param max
 */
void intensity_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                            float min,
                            float max);

/**
 * @brief neighborhood_downsampling
 * @param cloud
 * @param downsampled
 * @param radius
 * @param num_points
 */
void neighborhood_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                               float radius,
                               std::size_t num_points);

void horizontal_cloud_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                                   float height,
                                   std::size_t num_points);

void horizontal_section_cloud_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                                           float height,
                                           float thickness);

} // namespace numerix

/** @}*/

#endif //_NUMERIX_PCL_DOWNSAMPLING_

#endif // MYLIB_USE_PCL
