#ifdef _MYLIB_USE_PCL

#include "mlib/pcl/io.h"

#include "mlib/utility/string.h"

#ifdef _MYLIB_USE_PCL_WITH_VTK
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#endif // _MYLIB_USE_PCL_WITH_VTK

#include <iostream>
#include <fstream>
#include <iomanip>      // std::setprecision

namespace _mlib
{

#ifdef _MYLIB_USE_PCL_WITH_VTK
  void
  save_pcd(const std::string& filename,
           pcl::PointCloud<pcl::PointXYZI>& cloud)
  {
    if(pcl::io::savePCDFileASCII(filename, cloud)==-1)
      {
        std::cout << " ERROR! : cloud without data!" << std::endl;
      }
    return;
  }

  void
  save_ply(const std::string& filename,
           pcl::PointCloud<pcl::PointXYZI>& cloud)
  {
    if(pcl::io::savePLYFile(filename, cloud, true)==-1)
      {
        std::cout << " ERROR! : cloud without data!" << std::endl;
      }
    return;
  }
#endif // _MYLIB_USE_PCL_WITH_VTK

  void load_TXT_file(const std::string& filename,
                     pcl::PointCloud<pcl::PointXYZI>& cloud,
                     const std::string& separator)
  {
    std::ifstream fs;
    fs.open(filename.c_str(), std::ios::binary);
    if(!fs.is_open() || fs.fail())
      {
#ifdef _MYLIB_USE_PCL_WITH_VTK
        PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(),
                  strerror(errno));
#endif // _MYLIB_USE_PCL_WITH_VTK
        fs.close();
        return;
      }
    std::string line;
    std::vector<std::string> st;

    while(!fs.eof())
      {
        getline(fs, line);

        if(line == "")
          continue;
        st = _mlib::split(line,separator,true);

        // This allows to read RGB, too
        if(st.size() < 4)
          continue;

        pcl::PointXYZI pt;
        pt.x = double (atof(st[0].c_str()));
        pt.y = double (atof(st[1].c_str()));
        pt.z = double (atof(st[2].c_str()));
        pt.intensity = double (atof(st[3].c_str()));


        cloud.push_back(pt);
      }
    fs.close();

    cloud.width = uint32_t (cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
    return;
  };

  void
  save_txt(const std::string& qs_filename,
           pcl::PointCloud<pcl::PointXYZI>& cloud,
           const std::string& separator)
  {
    std::ofstream filename_out;
    filename_out << std::fixed << std::setprecision(10);
    filename_out.open(qs_filename);

    for(auto q: cloud)
      {
        filename_out << q.x
                     << separator
                     << q.y
                     << separator
                     << q.z
                     << separator
                     << q.intensity
                     << std::endl;
      }
    filename_out.close();
    return;
  }

  void
  save_txt(const std::string& qs_filename,
           pcl::PointCloud<pcl::PointXYZRGB>& cloud,
           const std::string& separator)
  {
    std::ofstream filename_out;
    filename_out << std::fixed << std::setprecision(10);
    filename_out.open(qs_filename);

    for(auto q: cloud)
      {
        filename_out << q.x
                     << separator
                     << q.y
                     << separator
                     << q.z
                     << separator
                     << q.r
                     << separator
                     << q.g
                     << separator
                     << q.b
                     << std::endl;
      }
    filename_out.close();
    return;
  }
}

#endif // _MYLIB_USE_PCL
