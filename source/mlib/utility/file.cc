#include "mlib/utility/file.h"

#include <fstream>      // std::ifstream

namespace mlib
{

  std::map<std::string, std::string>
  get_ext_and_name(const std::string& path)
  {
    std::map<std::string, std::string> map;
    std::string str(path);

    size_t sep = str.find_last_of("\\/");
    if(sep != std::string::npos)
      {
        map["dir"] = str.substr(0, sep);
        str = str.substr(sep + 1, str.size() - sep - 1);
      }
    else
      {
        map["dir"] = "./";
      }

    size_t dot = str.find_last_of(".");
    if(dot != std::string::npos)
      {
        map["name"] = str.substr(0, dot);
        map["ext"]  = str.substr(dot+1, str.size() - dot);
      }
    else
      {
        map["name"] = str;
        map["ext"]  = "";
      }
    return map;
  }

  bool
  file_exists(const std::string& path)
  {
    std::ifstream infile(path);
    bool result = infile.good();
    infile.close();
    return result;
  }

  std::size_t
  get_number_of_lines(const std::string& in_filename)
  {
    std::size_t n_lines = 0;

    std::string line;
    std::ifstream file(in_filename);
    while(getline(file, line))
      n_lines ++;

    return n_lines;
  }
}
