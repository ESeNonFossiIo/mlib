#ifdef MLIB_USE_PCL

#include "mlib/pcl/bounding_box.h"
#include "mlib/pcl/math.h"

#include <Eigen/Eigenvalues>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl/common/io.h>

namespace mlib
{

  BoundingBox::
  BoundingBox()
    :
    bounding_box(new pcl::PointCloud<pcl::PointXYZI>),
    projectionTransform(Eigen::Matrix4f::Identity())
  {}

  BoundingBox::
  BoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
    :
    bounding_box(new pcl::PointCloud<pcl::PointXYZI>),
    projectionTransform(Eigen::Matrix4f::Identity())
  {
    update(input_cloud);
  }

  void
  BoundingBox::
  update(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_points(new
                                                         pcl::PointCloud<pcl::PointXYZI>);
    copyPointCloud(*input_cloud, *original_points);  // NOTE controllare se serve

    // Compute principal directions
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input_cloud, centroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*input_cloud, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,
                                                                Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    // This line is necessary for proper orientation in some cases.
    //  The numbers come out the same without it, but the signs are different
    //  and the box doesn't get correctly oriented in some cases.

    pcl::PointCloud<pcl::PointXYZI>::Ptr projected_points(new
                                                          pcl::PointCloud<pcl::PointXYZI>);
    // Transform the original cloud to the origin where the principal components correspond to the axes.

    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,
                                                                                 0) * centroid.head<3>());

    pcl::transformPointCloud(*original_points, *projected_points,
                             projectionTransform);


    pcl::getMinMax3D(*projected_points, minPoint, maxPoint);

    pcl::PointCloud<pcl::PointXYZI>::Ptr extreme_point(new
                                                       pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr extreme_point_old(new
                                                           pcl::PointCloud<pcl::PointXYZI>);

    extreme_point->push_back(minPoint);
    extreme_point->push_back(maxPoint);
    for(size_t i = 0; i<2; ++i)
      for(size_t j = 0; j<2; ++j)
        for(size_t k = 0; k<2; ++k)
          {
            pcl::PointXYZI p;
            p.x = extreme_point->points[i].x;
            p.y = extreme_point->points[j].y;
            p.z = extreme_point->points[k].z;
            extreme_point_old -> push_back(p);
          }
    projectionTransform_inv = projectionTransform.inverse();
    pcl::transformPointCloud(*extreme_point_old, *bounding_box,
                             projectionTransform_inv);

    pcl::compute3DCentroid(*bounding_box, centroid);

    c.x = centroid[0];
    c.y = centroid[1];
    c.z = centroid[2];
  }

  void
  BoundingBox::
  points_in_the_bb
  (pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
   pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud,
   bool inside)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_points(new
                                                         pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_points(new
                                                            pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new
                                                pcl::PointCloud<pcl::PointXYZI>);
    copyPointCloud(*input_cloud, *original_points); // NOTE controllare se serve

    out_cloud->clear();
    pcl::transformPointCloud(*original_points, *transformed_points,
                             projectionTransform);


    bool point_is_inside;
    for(auto q: *transformed_points)
      {
        if((q.x >= minPoint.x) && (q.x <= maxPoint.x) &&
           (q.y >= minPoint.y) && (q.y <= maxPoint.y) &&
           (q.z >= minPoint.z) && (q.z <= maxPoint.z))
          {
            point_is_inside = true;
          }
        else
          {
            point_is_inside = false;
          }

        if(inside && point_is_inside)
          output -> push_back(q);

        if((!inside) && (!point_is_inside))
          output -> push_back(q);

      }
    pcl::transformPointCloud(*output, *out_cloud,
                             projectionTransform_inv);

  }

  Segment
  BoundingBox::
  get_dominant_direction()
  {
    pcl::PointXYZI a,b;
    double max = 0;

    for(size_t p  = 0; p < 3; ++p)
      {
        size_t pp = pow(2, p);
        pcl::PointCloud<pcl::PointXYZI>::Ptr
        even(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr
        odd(new pcl::PointCloud<pcl::PointXYZI>);
        for(size_t id = 0; id < 8; ++id)
          {
            if(((id%(2*pp))/(int)pp) == 0)
              {
                even->push_back(bounding_box->points[id]);
              }
            else
              {
                odd->push_back(bounding_box->points[id]);
              }
          }
        pcl::PointXYZI c_even,c_odd;
        pcl::computeCentroid(*even, c_even);
        pcl::computeCentroid(*odd, c_odd);

        if(points_distance(c_even, c_odd) > max)
          {
            max = points_distance(c_even, c_odd);
            a = c_even;
            b = c_odd;
          }
      }
    return Segment(a,b);
  }

  double
  BoundingBox::
  diameter()
  {
    double d(0.0);
    for(auto q: *bounding_box)
      for(auto p: *bounding_box)
        {
          double qp = points_distance(q,p);
          if(qp > d)
            d = qp;
        }
    return d;
  }

  std::vector<double>
  BoundingBox::
  sizes()
  {
    std::vector<double> s;

    double max = 0;
    for(size_t p  = 0; p < 3; ++p)
      {
        size_t pp = pow(2, p);
        pcl::PointCloud<pcl::PointXYZI>::Ptr
        even(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr
        odd(new pcl::PointCloud<pcl::PointXYZI>);
        for(size_t id = 0; id < 8; ++id)
          {
            if(((id%(2*pp))/(int)pp) == 0)
              {
                even->push_back(bounding_box->points[id]);
              }
            else
              {
                odd->push_back(bounding_box->points[id]);
              }
          }
        pcl::PointXYZI c_even,c_odd;
        pcl::computeCentroid(*even, c_even);
        pcl::computeCentroid(*odd,  c_odd);

        s.push_back(points_distance(c_even, c_odd));
      }

    std::sort(s.begin(), s.end(), std::greater<double>());

    return s;
  }


  STATUS
  BoundingBox::
  is_flat(double tolerance)
  {
    std::vector<int> counter(8,0);
    for(unsigned int i = 0; i<8; i++)
      {
        auto q = bounding_box->points[i];
        for(unsigned int j = i+1; j<8; j++)
          {
            auto p = bounding_box->points[j];
            double qp = points_distance(q,p);
            // std::cout << "--->" << qp << std::endl;
            if(qp < tolerance)
              {
                counter[i]++;
                counter[j]++;
              }
          }
      }
    for(unsigned int i = 0; i<8; i++)
      {
        if(counter[i] != 1)
          return STATUS::MLIB_ERROR;
      }
    return STATUS::MLIB_SUCCEED;
  }
}

#endif // MLIB_USE_PCL
