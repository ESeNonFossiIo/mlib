#include "mlib/utility/logger.h"

#include <iostream>
#include <iomanip>  // std::setprecision

#include <stdio.h>
#ifdef WIN32
#if _MSC_VER < 1900
#define snprintf _snprintf
#endif
#endif

namespace _mlib
{

  using namespace _mlib::Color;

  Logger::
  Logger(const std::string& filename_,
         const bool& write_on_stdcout_)
    :
    filename(filename_),
    write_on_stdcout(write_on_stdcout_)
  {
    if(filename != "")
      {
        write_on_file = true;
      }
    else
      {
        write_on_file = false;
      }
    begin  = clock();
  };

  Logger::
  Logger(const Logger& copy)
  {
    content = copy.content;
  };

  Logger::
  ~Logger()
  {};

  void
  Logger::
  save_on_file(const std::string& filename_)
  {
    if(filename_ != "")
      {
        filename = filename_;
        write_on_file = true;
      }
    if(write_on_file)
      {
        file.open(filename);
        for(std::string l: content)
          {
            file << l
                 << std::endl;
          }
        file.close();
      }
  }

  void
  Logger::
  print()
  {
    for(std::string l: content)
      {
        std::cout << l
                  << std::endl;
      }
  }

  void
  Logger::
  write(const std::string& msg,
        const std::string& str,
        const GeneralColor& color)
  {
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    auto format = "[%15.3f] %10s - %s";
#ifndef WIN32
    size_t size = std::snprintf(nullptr, 0, format, elapsed_secs, msg.c_str(),
                                str.c_str());
#else
    size_t size = snprintf(nullptr, 0, format, elapsed_secs, msg.c_str(),
                           str.c_str());
#endif

    std::string output(size + 1, '\0');
    std::sprintf(&output[0], format, elapsed_secs, msg.c_str(), str.c_str());

    output = color.init() + output + color.end();

    content.push_back(output);

    if(write_on_stdcout)
      std::cout << output
                << std::endl;

    return;
  };

  void
  Logger::
  warning(const std::string& str)
  {
    write("WARNING", str, yellow);
  };

  void
  Logger::
  error(const std::string& str)
  {
    write("ERROR", str, red);
  };

  void
  Logger::
  status(const std::string& str)
  {
    write("STATUS", str, blue);
  };

  void
  Logger::
  value(const std::string& str, const std::string& val)
  {
    auto format = "%20s = %20s";

#ifndef WIN32
    size_t size = std::snprintf(nullptr, 0, format,
                                str.c_str(), val.c_str());
#else
    size_t size = sprintf_s(nullptr, 0, format,
                            str.c_str(), val.c_str());
#endif

    std::string output(size + 1, '\0');
    std::sprintf(&output[0], format, str.c_str(), val.c_str());

    write("VALUE", output, green);
  };

  void
  Logger::
  msg(const std::string& str)
  {
    write("MSG", str, white);
  };

}
