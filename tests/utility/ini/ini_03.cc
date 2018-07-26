#include "../../test.h"

#include "mlib/utility/parser/ini.h"

#include <iostream>

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

  INIHandler test;

  test.add_entry("section1", "entry1", "val1");
  test.add_entry("section1", "entry2", "val2");
  test.add_entry("section2", "entry1", "val1");
  test.add_entry("section2", "entry2", "val2");

  std::ostream out(std::cout.rdbuf());
  test.save(out);

  std::cout << test["section2"]["entry2"] << std::endl;
  test["section2"]["entry2"]= "new_val";
  std::cout << test["section2"]["property1"] << std::endl;
  return 0;
}
