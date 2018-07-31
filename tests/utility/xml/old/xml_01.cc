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


  XMLHandler test(get_test_dir()+"/utility/xml/xml_01.xml");
  test.print();
  std::cout
      << "====================" << std::endl
      << test.get_file() << std::endl
      << "====================" << std::endl;
  // test.save(get_test_dir()+"/utility/xml_01.xml");
  std::cout << test.get_element(0)[0][0]->get_val();
  return 0;
}
