#include "mlib/utility/file.h"
#include "mlib/utility/string.h"

#include <map>

std::string get_test_dir()
{

  std::map<std::string, std::string> ext_name =
    _mlib::get_ext_and_name(__FILE__);
  return ext_name["dir"];
}

void print_title(const std::string& title)
{
  std::cout << std::endl <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for " << title << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl << std::endl;
}

void make_test_pass(const std::string& filename)
{
  std::string fn(get_test_dir()+"/"+filename+".output");
  std::cout << _mlib::get_file_content(fn);
}
