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

  test.check();
  test.rm_entry("section1", "entry1");
  test.check();
  test.rm_entry("section1", "entry2");
  test.check();

  return 0;
}
