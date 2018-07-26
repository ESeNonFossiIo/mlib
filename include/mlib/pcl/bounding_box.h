#ifdef MLIB_USE_PCL

#ifndef _MLIB_PCL_BOUNDING_BOX_
#define _MLIB_PCL_BOUNDING_BOX_

#include <mlib/utility/status.h>
#include <mlib/math/geometry/segment.h>

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** \addtogroup PCL
 *  @{
 */

namespace mlib
{

  class BoundingBox
  {

  public:
    BoundingBox();

    BoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    void update(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    void points_in_the_bb
    (pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
     bool inside = true);

    double
    diameter();

    std::vector<double>
    sizes();

    STATUS
    is_flat(double tolerance = 0.01);

    Segment
    get_dominant_direction();

    pcl::PointCloud<pcl::PointXYZI>::Ptr bounding_box;
    pcl::PointXYZI c;

  private:
    // Min and max in the trasformete reference system
    pcl::PointXYZI minPoint, maxPoint;
    // Trasformation in th the reference system with eigen vectors as base
    Eigen::Matrix4f projectionTransform;
    Eigen::Matrix4f projectionTransform_inv;
  };

};


/** @}*/

#endif //_MLIB_PCL_BOUNDING_BOX_

#endif //MYLIB_USE_PCL
