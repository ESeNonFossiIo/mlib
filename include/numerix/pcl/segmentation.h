#ifdef NUMERIX_USE_PCL

#ifndef _NUMERIX_PCL_SEGMENTATION_
#define _NUMERIX_PCL_SEGMENTATION_

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
/** \addtogroup PCL
 *  @{
 */

namespace numerix {
pcl::ModelCoefficients fitting_plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                     const double& model_tolerance = 0.01);

}

/** @}*/

#endif //_NUMERIX_PCL_SEGMENTATION_

#endif // MYLIB_USE_PCL
