#include "mlib/utility/string.h"

// per std::stold
#include <cstdlib>
// std::remove_if, std::transform
#include <algorithm>
//std::ostringstream
#include <sstream>

#include <string>
#include <fstream>
#include <iostream>

namespace _mlib
{
  std::string lower_case(const std::string& in)
  {
    std::string out(in);
    std::transform(out.begin(), out.end(), out.begin(), ::tolower);
    return out;
  }

  std::string upper_case(const std::string& in)
  {
    std::string out(in);
    std::transform(out.begin(), out.end(), out.begin(), ::toupper);
    return out;
  }

  double
  from_str_to_double(const std::string in)
  {
    try
      {
        return std::stold(in.c_str());
      }
    catch(...)
      {
        return 0.0;
      }
  }

  std::string
  from_double_to_str(double Number)
  {
    std::ostringstream convert;
    convert << Number;
    return convert.str();
  }

  unsigned int
  from_str_to_unsigned_int(const std::string in)
  {
    try
      {
        return std::stoul(in.c_str());
      }
    catch(...)
      {
        return 0;
      }
  }

  int
  from_str_to_int(const std::string in)
  {
    try
      {
        return std::stoi(in.c_str());
      }
    catch(...)
      {
        return 0;
      }
  }

  bool
  from_str_to_bool(const std::string in)
  {
    std::string tmp_in(in);
    std::vector<std::string> true_names = {"True", "true", "T", "t", "Yes", "yes", "Y", "y"};
    std::vector<std::string>::iterator it =  true_names.begin();
    std::vector<std::string>::iterator end =  true_names.end();

    trim(tmp_in);
    for(; it!=end; ++it)
      if(*it==tmp_in) return true;

    return false;
  }

  std::string
  remove_white_spaces(std::string& str)
  {
    str.erase(std::remove_if(str.begin(), str.end(), ::isspace),
              str.end());
    return str;
  }

  std::string
  trim(std::string& str)
  {
    str.erase(std::remove(str.begin(), str.end(), '\n'),
              str.end());
    return remove_white_spaces(str);
  }

  std::vector<std::string>
  split(const std::string& str,
        const std::string& delim,
        const bool remove_white_space,
        const bool allow_empty_fields)
  {
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
      {
        pos = str.find(delim, prev);
        if(pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);

        if(remove_white_space)
          _mlib::remove_white_spaces(token);

        if(!token.empty() || allow_empty_fields) tokens.push_back(token);
        prev = pos + delim.length();
      }
    while(pos < str.length() && prev < str.length());
    return tokens;
  }

  std::string
  hline(const unsigned int& size, const std::string& c)
  {
    std::string out = "";
    for(unsigned int i = 0; i < size; ++i)
      out += c;
    return out;
  }

  std::string get_file_content(std::string& filename)
  {
    std::ifstream myfile(filename);
    std::string output((std::istreambuf_iterator<char>(myfile)),
                       std::istreambuf_iterator<char>());
    return output;
  }
}
