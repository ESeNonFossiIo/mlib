#ifdef MLIB_USE_PCL

#include "mlib/pcl/cluster.h"

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

namespace mlib
{

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
  get_clusters(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const double& tolerance,
    const unsigned int& min_pts_per_cluster)
  {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> result;

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::PointCloud<pcl::PointXYZI>::Ptr noNanCloud(new
                                                    pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*noNanCloud,indices);
    size_t max_pts_per_cluster = noNanCloud->size();

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new
                                                  pcl::search::KdTree<pcl::PointXYZI> ());

    tree->setInputCloud(noNanCloud);
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_pts_per_cluster);
    ec.setMaxClusterSize(max_pts_per_cluster);
    ec.setSearchMethod(tree);
    ec.setInputCloud(noNanCloud);
    ec.extract(cluster_indices);

    for(auto it = cluster_indices.begin();
        it != cluster_indices.end();
        ++it)
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new
                                                           pcl::PointCloud<pcl::PointXYZI>);

        for(std::vector<int>::const_iterator pit = it->indices.begin();
            pit != it->indices.end(); ++pit)
          {
            cloud_cluster->points.push_back(cloud->points[*pit]);
          }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;


        result.push_back(cloud_cluster);
      }
    return result;
  }

}

#endif // MLIB_USE_PCL
