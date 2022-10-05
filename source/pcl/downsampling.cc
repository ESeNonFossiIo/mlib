#ifdef MLIB_USE_PCL

#include "mlib/pcl/downsampling.h"

#include "mlib/pcl/bounding_box.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/norms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>

namespace mlib
{
  void
  voxel_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                     float cube_size)
  {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(cube_size, cube_size, cube_size);
    vg.filter(*downsampled);
    return;
  }

  void
  intensity_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                         float min, float max)
  {
    pcl::PassThrough<pcl::PointXYZI> pass_through;
    pass_through.setInputCloud(cloud);
    pass_through.setFilterFieldName("intensity");
    pass_through.setFilterLimits(min, max);
    pass_through.filter(*downsampled);
    return;
  }

  void
  neighborhood_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                            float radius, std::size_t num_points)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_outlier_removal;
    radius_outlier_removal.setInputCloud(cloud);
    radius_outlier_removal.setRadiusSearch(radius);
    radius_outlier_removal.setMinNeighborsInRadius(num_points);
    radius_outlier_removal.filter(*downsampled);
    return;
  }

  void
  horizontal_cloud_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                                float height, std::size_t num_points)
  {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    for(auto q: *cloud)
      {
        if(q.z<min)
          min = q.z;
        if(q.z>max)
          max = q.z;
      }

    double l = max-min;
    int I = (int)(l/height) + 1;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> sections;
    for(int i = 0; i < I; ++i)
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_section(new
                                                         pcl::PointCloud<pcl::PointXYZI>);
        sections.push_back(new_section);
      }

    for(auto q: *cloud)
      {
        int iq = (int)((q.z - min)/height);
        sections[iq]->push_back(q);
      }

    downsampled->clear();
    for(int i = 0; i < I; ++i)
      {
        if(sections[i]->size()>num_points)
          *downsampled+=*sections[i];
      }
    return;
  }

  void
  horizontal_section_cloud_downsampling(pcl::PointCloud<pcl::PointXYZI>::Ptr
                                        cloud,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled,
                                        float height, float thickness)
  {
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    for(auto q: *cloud)
      {
        if(q.z<min)
          min = q.z;
        if(q.z>max)
          max = q.z;
      }

    double l = max-min;
    int I = (int)(l/height) + 1;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> sections;
    for(int i = 0; i < I; ++i)
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr new_section(new
                                                         pcl::PointCloud<pcl::PointXYZI>);
        sections.push_back(new_section);
      }

    for(auto q: *cloud)
      {
        int iq = (int)((q.z - min)/height);
        sections[iq]->push_back(q);
      }

    downsampled->clear();
    for(int i = 0; i < I; ++i)
      {
        BoundingBox bb(sections[i]);

        if(bb.diameter() > thickness)
          *downsampled+=*sections[i];
      }
    return;
  }
}

#endif // MLIB_USE_PCL
