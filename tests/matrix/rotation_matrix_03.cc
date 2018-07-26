#include "mlib/math/angle.h"
#include "mlib/math/constants.h"
#include "mlib/math/matrix/rotation.h"

#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Matrix" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;


  Angle roll(0.123);
  Angle pitch(-0.123);
  Angle yaw(1.13);

  TaitBryanAngles angles(roll, pitch, yaw);

  RotationMatrix R(Point(1,0,0), roll);
  RotationMatrix P(Point(0,1,0), pitch);
  RotationMatrix Y(Point(0,0,1), yaw);

  auto M = RotationMatrix(angles);
  auto N = RotationMatrix(angles, RotationType::ZYX);

  std::cout <<  "check .... " << (M - (R*P*Y)).l_inf_norm() <<
            std::endl;
  std::cout <<  "check .... " << (N - (Y*P*R)).l_inf_norm() <<
            std::endl;

  std::cout <<  "check .... " << (M).get_angles().roll(AngleType::deg) <<
            std::endl;
  std::cout <<  "check .... " << (N).get_angles(RotationType::ZYX).roll(
              AngleType::deg) <<
            std::endl;
}
