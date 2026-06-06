#ifdef NUMERIX_USE_PCL

#ifndef _NUMERIX_PCL_BOUNDING_BOX_
#define _NUMERIX_PCL_BOUNDING_BOX_

#include <numerix/math/geometry/segment.h>
#include <numerix/utility/status.h>

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/** \addtogroup PCL
 *  @{
 */

namespace numerix {

class BoundingBox {

public:
    BoundingBox();

    BoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    void update(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    void points_in_the_bb(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
                          bool inside = true);

    double diameter();

    std::vector<double> sizes();

    STATUS
    is_flat(double tolerance = 0.01);

    Segment get_dominant_direction();

    pcl::PointCloud<pcl::PointXYZI>::Ptr bounding_box;
    pcl::PointXYZI c;

private:
    // Min and max in the trasformete reference system
    pcl::PointXYZI minPoint, maxPoint;
    // Trasformation in th the reference system with eigen vectors as base
    Eigen::Matrix4f projectionTransform;
    Eigen::Matrix4f projectionTransform_inv;
};

} // namespace numerix

/** @}*/

#endif //_NUMERIX_PCL_BOUNDING_BOX_

#endif // MYLIB_USE_PCL
