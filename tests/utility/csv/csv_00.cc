#include "../../test.h"

#include "mlib/utility/parser/csv.h"
#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for utility - CSVHandler" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  CSVHandler test(get_test_dir()+"/utility/csv/csv_00.csv");

  std::vector<std::string> labels = test.get_labels();
  for(unsigned int i = 0; i < labels.size(); ++i)
    std::cout <<  " --> " <<labels[i] << std::endl;

  std::vector<std::string> datum = test.get("title1");
  for(unsigned int i = 0; i < datum.size(); ++i)
    std::cout <<  " ---> " <<datum[i] << std::endl;
  return 0;
}
