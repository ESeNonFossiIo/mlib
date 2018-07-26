#include "../test.h"

#include "mlib/utility/config_file.h"
#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - ParsedParameters" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

//  ConfigFile cf(get_test_dir()+"/utility/config_file_01.ini");
// cf.check();

  ParsedParameters test(get_test_dir()+"/utility/config_parser_00.ini");
  double d        = test.add_new_entry("section1","double1",3.14);
  unsigned int u  = test.add_new_entry("section1","prova",(unsigned int)3);
  int v           = test.add_new_entry("section2","prova",4);
  std::cout <<  "section1.prova = "  << u << std::endl;
  std::cout <<  "section2.prova = "  << v << std::endl;
  test.check();
  return 0;
}
