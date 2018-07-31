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


  XMLHandler manager;
  manager.set_style("prova.css");
  manager.set_xml_header("version", "1.0");
  manager.set_xml_stylesheet("type", "text/xsl");

  XMLEntry test("sezione","prova");
  test.add_property("property1", "val1");
  test.add_property("property2", "val2");

  XMLEntry test2("subsezione","prova");
  test2.add_property("subproperty1", "val1");

  XMLEntry test3("subsubsezione","prova");
  test3.add_property("subsubproperty1", "val1");

  test2.add(test3);
  test.add(test2);

  // std::shared_ptr<XMLEntry> new_entry = std::make_shared<XMLEntry>("subsezione","prova");

  manager.add(test);

  std::cout << manager.get_file();
  manager.save(get_test_dir()+"/utility/xml/xml_00.xml");
  return 0;
}
