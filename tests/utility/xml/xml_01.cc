#include "../../test.h"

#include "mlib/utility/parser/xml.h"
#include <iostream>

using namespace mlib;

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


  XMLHandler manager(get_test_dir()+"/utility/xml/xml_01.xml");
  manager.print();
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << manager["sezione"]["subsezione"]();
  std::cout <<
            "=================================================" <<
            std::endl;
  return 0;
}
