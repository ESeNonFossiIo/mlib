#include "mlib/utility/file.h"

#include <iostream>
#include <map>

using namespace mlib;

void test(std::string filename)
{
  std::map<std::string, std::string> ext_name =
    get_ext_and_name(filename);
  std::cout << "filename ......... " << filename << std::endl;
  std::cout << "dir .............. " << ext_name["dir"] <<
            std::endl;
  std::cout << "name ............. " << ext_name["name"] <<
            std::endl;
  std::cout << "ext .............. " << ext_name["ext"] <<
            std::endl;
  std::cout << std::endl;
}

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - get_ext_and_name" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  test("test.cpp");
  test("/usr/bin/testt.cpp");
  test("/usr/bin/testt");
  test("/usr/bin/tes.tt.txt");

  return 0;
}
