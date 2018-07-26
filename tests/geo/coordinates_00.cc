#include "../test.h"
#include "../test_compare.h"

#include "mlib/geo/coordinates.h"

#include <iostream>
#include <vector>

using namespace _mlib;

int main()
{

  print_title("Coordinates 00");

  double lat  = 42.497812;
  double lon  = 12.153571;

  std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
  std::cout.precision(9);

  Point p = UTM_latlon_to_xy(lat, lon);

  std::cout << p;
}
