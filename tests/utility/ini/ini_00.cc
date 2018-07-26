#include "../../test.h"

#include "mlib/utility/parser/ini.h"
#include <iostream>

// void test(std::string filename)
// {
//   std::pair<std::string, std::string> ext_name = get_ext_and_name(filename);
//   std::cout << "filename ......... " << filename << std::endl;
//   std::cout << "name ............. " << ext_name.first << std::endl;
//   std::cout << "ext .............. " << ext_name.second << std::endl;
//   std::cout << std::endl;
// }

using namespace _mlib;

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


  INIHandler test("[section1]\n\
                    prova=2\n\
                  test=3\ncontinua\n\
                    [section2]\n\
                    prova=2\n\
                    [section3]\n\
                    prova=3", false);

  std::cout <<  "--->" << test.get_sections()[0]<<std::endl;
  std::cout <<  "--->" << test.get_sections()[1]<<std::endl;
  std::cout <<  "--->" << test.get_sections()[2]<<std::endl;
  std::cout <<  "--->" <<
            test.get_entries("section1")[0]<<std::endl;
  std::cout <<  "--->" <<
            test.get_entries("section1")[1]<<std::endl;
  std::cout <<  "--->" << test.get_sections().size()
            <<std::endl;
  std::cout <<  "--->" << test.get_val<std::string>("section1",
                                                    "prova")<<std::endl;

  test.check();
  return 0;
}
