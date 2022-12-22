#ifdef MLIB_USE_PCL

#include "mlib/pcl/segmentation.h"

namespace mlib
{
  pcl::ModelCoefficients fitting_plane(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const double& model_tolerance)
  {
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(model_tolerance);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, coefficients);
    return coefficients;
  }
}

#endif // MLIB_USE_PCL
