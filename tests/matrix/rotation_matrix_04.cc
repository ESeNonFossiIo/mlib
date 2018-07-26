#include "mlib/math/angle.h"
#include "mlib/math/constants.h"
#include "mlib/math/matrix/rotation.h"

#include <iostream>
#include <vector>

using namespace _mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Matrix" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;


  Angle roll(0, AngleType::rad);
  Angle pitch(1.3, AngleType::rad);
  Angle yaw(2.0, AngleType::rad);

  TaitBryanAngles angles(roll, pitch, yaw);

  auto M = RotationMatrix(angles);
  auto angles_from_matrix = get_roll_pitch_yaw(M);

  std::cout <<  "r .... " << angles_from_matrix.roll() <<
            std::endl;
  std::cout <<  "p .... " << angles_from_matrix.pitch() <<
            std::endl;
  std::cout <<  "y .... " << angles_from_matrix.yaw() <<
            std::endl;
  return 0;
}
