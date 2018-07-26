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

  INIHandler test;

  test.add_entry("section1", "entry1", "val1");
  test.add_entry("section1", "entry2", "val2");
  test.add_entry("section2", "entry1", "val1");
  test.add_entry("section2", "entry2", "val2");

  auto l = test.get_logger("log.txt");
  l.save_on_file();

  return 0;
}
