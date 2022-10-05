#ifndef _MLIB_LICENSE_HANDLER_H__
#define _MLIB_LICENSE_HANDLER_H__

#include "mlib/utility/string.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/hdreg.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib> // for exit()

#include <cipher/cipher.h>
#include <algorithm>    // std::remove_if

namespace mlib
{
  std::string get_serial_no()
  {
    static struct hd_driveid hd;
    int fd;

    if(geteuid() >  0)
      {
        printf("ERROR: Must be root to use\n");
        exit(1);
      }

    if((fd = open("/dev/sda",
                  O_RDONLY|O_NONBLOCK)) < 0) //TODO: usare un device generico
      {
        printf("ERROR: Cannot open device %s\n", "/dev/sda");
        exit(1);
      }

    if(!ioctl(fd, HDIO_GET_IDENTITY, &hd))
      {
        // printf("Hard Disk Model: %.40s\n", hd.model);
        // printf("  Serial Number: %.20s\n", hd.serial_no);
      }
    else if(errno == -ENOMSG)
      {
        printf("No hard disk identification information available\n");
      }
    else
      {
        perror("ERROR: HDIO_GET_IDENTITY");
        exit(1);
      }

    std::string r(reinterpret_cast<char*>(hd.serial_no));

    r.erase(std::remove(r.begin(),r.end(),' '),r.end());

    return r;
  }

  template<typename CIPHER>
  class LicenseHandler
  {
  public:
    LicenseHandler(const std::string& filename_,
                   const CIPHER& cipher_)
      :
      filename(filename_),
      cipher(cipher_)
    {
      license = get_serial_no();

      license+=license;
      license+=license;
      license+=license; // TODO: generalizzare
    }

    void
    generate_license() const
    {
      std::ofstream myfile;
      myfile.open(filename);
      myfile << this->cipher.encrypt(license);
      myfile.close();
      return;
    }

    void
    read_license_raw(const std::string& license_file) const
    {
      license = get_file_content(license_file);
      return;
    }

    void
    generate_raw_license() const
    {
      std::ofstream myfile;
      myfile.open(filename);
      myfile << license;
      myfile.close();
      return;
    }

    bool
    check_license() const
    {
      std::string output = get_file_content(filename);
      return (license.compare(this->cipher.decrypt(output)) == 0);
    }

    std::string filename;
    mutable std::string license;
    CIPHER cipher;
  };
}
#endif //_MLIB_LICENSE_HANDLER_H__
