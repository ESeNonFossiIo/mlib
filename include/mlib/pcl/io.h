#ifdef _MYLIB_USE_PCL

#ifndef __MYLIB_PCL_IO_WRAPPER_
#define __MYLIB_PCL_IO_WRAPPER_

#include <iostream>
#ifdef _MYLIB_USE_PCL_WITH_VTK
#include <pcl/io/pcd_io.h>
#endif //_MYLIB_USE_PCL_WITH_VTK
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** \addtogroup PCL
 *  @{
 */

namespace _mlib
{

#ifdef _MYLIB_USE_PCL_WITH_VTK
  template<typename CLOUDTYPE = pcl::PointXYZ>
  void save_PCD(const std::string& filename,
                const pcl::PointCloud<CLOUDTYPE>& cloud)
  {
    pcl::io::savePCDFileASCII(filename, cloud);
  };

  template<typename CLOUDTYPE = pcl::PointXYZ>
  int load_PCD(const std::string& filename,
               pcl::PointCloud<CLOUDTYPE>& cloud)
  {
    return pcl::io::loadPCDFile<CLOUDTYPE> (filename, cloud);
  };


  void
  save_pcd(const std::string& qs_filename,
           pcl::PointCloud<pcl::PointXYZI>& cloud);

  void
  save_ply(const std::string& qs_filename,
           pcl::PointCloud<pcl::PointXYZI>& cloud);

#endif //_MYLIB_USE_PCL_WITH_VTK

  void
  load_TXT_file(const std::string& filename,
                pcl::PointCloud<pcl::PointXYZI>& cloud,
                const std::string& separator = ";");

  void
  save_txt(const std::string& qs_filename,
           pcl::PointCloud<pcl::PointXYZI>& cloud,
           const std::string& separator = ";");

  void
  save_txt(const std::string& qs_filename,
           pcl::PointCloud<pcl::PointXYZRGB>& cloud,
           const std::string& separator = ";");
};


/** @}*/

#endif //__MYLIB_PCL_IO_WRAPPER_

#endif //MYLIB_USE_PCL
