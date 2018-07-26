#include "../../test.h"

#include "mlib/utility/file.h"

#include <iostream>
#include <map>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - get_number_of_lines" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  std::string filename("/utility/file/file_02.cc");

  std::cout << " filename = ........................... "
            << filename
            << std::endl;
  std::cout << " number of lines of this file = ....... "
            << get_number_of_lines(get_test_dir()+filename)
            << std::endl;;

  return 0;
}
