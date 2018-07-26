#include "../test.h"
#include "../test_compare.h"

#include "mlib/math/angle.h"
#include "mlib/math/constants.h"
#include "mlib/math/matrix/rotation.h"

#include <iostream>
#include <vector>

using namespace mlib;

Matrixd
computeTransformationMatrix(const double& roll,const double& pitch,
                            const double& yaw)
{
  Matrixd Mat(3,3);

  Mat(0,0) = cos(yaw) * cos(pitch);
  Mat(0,1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
  Mat(0,2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);

  Mat(1,0) = sin(yaw) * cos(pitch);
  Mat(1,1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
  Mat(1,2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);

  Mat(2,0) = -sin(pitch);
  Mat(2,1) = cos(pitch) * sin(roll);
  Mat(2,2) = cos(pitch) * cos(roll);

  return Mat;
}

double test_val(const double& r, const double& p, const double& y)
{
  Angle roll(r, AngleType::deg);
  Angle pitch(p, AngleType::deg);
  Angle yaw(y, AngleType::deg);

  TaitBryanAngles angles(roll, pitch, yaw);

  auto M = RotationMatrix(angles, RotationType::ZYX);
  return (M - computeTransformationMatrix(roll.rad(), pitch.rad(),
                                          yaw.rad())).l_2_norm();
}

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Matrix" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  for(int r = -90; r <= 90; r+=10)
    for(int p = -90; p <= 90; p+=10)
      for(int y = -90; y <= 90; y+=10)
        {
          static double val = test_val(r, p, y);
          std::cout <<  are_equal(val);
        }
}
