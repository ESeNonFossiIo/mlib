#include "../../test.h"

#include "mlib/utility/parser/xml_old.h"
#include <iostream>

using namespace _mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - XMLHandler" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;


  XMLHandler test(get_test_dir()+"/utility/xml/xml_02.xml");

  std::cout << test.get_file() << std::endl
            << "====================" << std::endl;
  return 0;
}
