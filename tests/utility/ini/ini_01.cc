#include "../../test.h"

#include "mlib/utility/parser/ini.h"
#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - INIHandler" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  INIHandler test(get_test_dir()+"/utility/ini/ini_01.ini");

  std::cout <<  "--->" << test.get_sections()[0]<<std::endl;
  std::cout <<  "--->" << test.get_sections()[1]<<std::endl;
  std::cout <<  "--->" << test.get_sections()[2]<<std::endl;
  std::cout <<  "--->" << test.get_entries("section1")[0]<<std::endl;
  std::cout <<  "--->" << test.get_entries("section1")[1]<<std::endl;
  std::cout <<  "--->" << test.get_sections().size()
            <<std::endl;
  std::cout <<  "--->" << test.get_val("section1",
                                       "prova")<<std::endl;
  test.check();
  return 0;
}
