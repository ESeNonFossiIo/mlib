#include "mlib/math/matrix/rotation.h"
#include "mlib/math/constants.h"

#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Angle" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  Angle angle(189,    AngleType::deg);
  RotationMatrix R(Point(sqrt(2) /2,sqrt(2) /2,0), angle);
  auto result = get_axis_and_angle(R);
  std::cout << "-------------------------" << std::endl;
  std::cout << result.first         << std::endl;
  std::cout << result.second.deg()  << std::endl;
  std::cout << "-------------------------" << std::endl;

  RotationMatrix R2(result.first, result.second);
  if((R - R2).l_inf_norm() < VAR_MLIB_ZERO_TOLERANCE)
    std::cout << "OK!" << std::endl;
  std::cout << "-------------------------" << std::endl;
}
