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
  std::cout << "  TEST for utility - file_exists" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  // std::cout << " DIRECTORY :      " << get_test_dir()+"/utility/file_01.output" <<
  // std::endl;
  std::cout << " file_01.output  :" << file_exists(get_test_dir()
                                                   +"/utility/file/file_01.output") << std::endl;
  std::cout << " file_01.outputn :" << file_exists(get_test_dir()
                                                   +"/utility/file/file_01.outputn") << std::endl;

  return 0;
}
