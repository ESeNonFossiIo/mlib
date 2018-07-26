#include "../../test.h"

#include "mlib/utility/parser/csv.h"
#include <iostream>

using namespace _mlib;

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

  CSVHandler test(get_test_dir()+"/utility/csv/csv_01.csv");

  std::vector<std::string> labels = test.get_labels();

  // for(unsigned int i = 0; i < labels.size(); ++i)
  //   std::cout << i << " --> " <<labels[i] << std::endl;

  std::vector<std::string> date = test.get("Date");
  std::vector<std::string> time = test.get("Time");

  std::vector<std::string> lat_txt = test.get("PosLat");
  std::vector<std::string> lon_txt = test.get("PosLon");
  std::vector<std::string> alt_txt = test.get("PosAlt");

  std::vector<std::string> yaw_txt   = test.get("AngleHeading");
  std::vector<std::string> pitch_txt = test.get("AnglePitch");
  std::vector<std::string> roll_txt  = test.get("AngleRoll");

  for(unsigned int i = 0; i < 10; ++i)
    {
      std::cout   << date[i]      << ";"
                  << time[i]      << ";"
                  << lat_txt[i]   << ";"
                  << lon_txt[i]   << ";"
                  << alt_txt[i]   << ";"
                  << yaw_txt[i]   << ";"
                  << pitch_txt[i] << ";"
                  << roll_txt[i]
                  << std::endl;
    }
  for(unsigned int i = roll_txt.size() - 10; i < roll_txt.size(); ++i)
    {
      std::cout   << date[i]      << ";"
                  << time[i]      << ";"
                  << lat_txt[i]   << ";"
                  << lon_txt[i]   << ";"
                  << alt_txt[i]   << ";"
                  << yaw_txt[i]   << ";"
                  << pitch_txt[i] << ";"
                  << roll_txt[i]
                  << std::endl;
    }

  return 0;
}
