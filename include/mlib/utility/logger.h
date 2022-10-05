#ifndef _MLIB_PCL_LOGGER
#define _MLIB_PCL_LOGGER

#include <fstream>
#include <ctime>
#include <vector>
#include <string>

#include "mlib/utility/color.h"

namespace mlib
{

  using namespace std;

  class Logger
  {
  public:
    Logger(const std::string& filename_ = "",
           const bool& write_on_stdcout_ = false);

    Logger(const Logger& copy);

    ~Logger();

    void
    save_on_file(const std::string& filename_ = "");

    void
    print();

    void write(const std::string& msg,
               const std::string& str,
               const GeneralColor& color);

    void warning(const std::string& str);

    void error(const std::string& str);

    void status(const std::string& str);

    void value(const std::string& str, const std::string& val);

    void msg(const std::string& str);

  private:
    clock_t     begin;
    std::string filename;
    ofstream    file;
    bool        write_on_stdcout;
    bool        write_on_file;

    std::vector<std::string> content;
  };


}
#endif //_MLIB_PCL_LOGGER
