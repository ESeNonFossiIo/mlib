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
  {
    Angle roll(1.225);
    Angle pitch(-1);
    Angle yaw(0.10);

    RotationMatrix R(Point(1,0,0), roll);
    RotationMatrix P(Point(0,1,0), pitch);
    RotationMatrix Y(Point(0,0,1), yaw);

    auto m = R * P * Y;
    auto angles = get_roll_pitch_yaw(m);

    RotationMatrix T(angles);
    std::cout << "check .... ";
    if((m - T).l_inf_norm() < VAR_MLIB_ZERO_TOLERANCE)
      std::cout << "OK!" << std::endl;
    std::cout << "-------------------------" << std::endl;
  }
  {
    Angle roll(2.25);
    Angle pitch(0);
    Angle yaw(1.10);

    RotationMatrix R(Point(1,0,0), roll);
    RotationMatrix P(Point(0,1,0), pitch);
    RotationMatrix Y(Point(0,0,1), yaw);

    auto m = R * P * Y;
    auto angles = get_roll_pitch_yaw(m);
    RotationMatrix T(angles);
    std::cout << "check .... ";
    if((m - T).l_inf_norm() < VAR_MLIB_ZERO_TOLERANCE)
      std::cout << "OK!" << std::endl;
    std::cout << "-------------------------" << std::endl;
  }

  {
    Angle roll(1.225);
    Angle pitch(M_PI);
    Angle yaw(-1.10);

    RotationMatrix R(Point(1,0,0), roll);
    RotationMatrix P(Point(0,1,0), pitch);
    RotationMatrix Y(Point(0,0,1), yaw);

    auto m = R * P * Y;
    auto angles = get_roll_pitch_yaw(m);
    RotationMatrix T(angles);
    std::cout << "check .... ";
    if((m - T).l_inf_norm() < VAR_MLIB_ZERO_TOLERANCE)
      std::cout << "OK!" << std::endl;
    std::cout << "-------------------------" << std::endl;
  }
  {
    Angle roll(1.225);
    Angle pitch(M_PI/2);
    Angle yaw(-1.10);

    RotationMatrix R(Point(1,0,0), roll);
    RotationMatrix P(Point(0,1,0), pitch);
    RotationMatrix Y(Point(0,0,1), yaw);

    auto m = R * P * Y;
    auto angles = get_roll_pitch_yaw(m);
    RotationMatrix T(angles);
    std::cout << "check .... ";
    if((m - T).l_inf_norm() < VAR_MLIB_ZERO_TOLERANCE)
      std::cout << "OK!" << std::endl;
    std::cout << "-------------------------" << std::endl;
  }
  {
    Angle roll(1.225);
    Angle pitch(-M_PI/2);
    Angle yaw(-1.10);

    RotationMatrix R(Point(1,0,0), roll);
    RotationMatrix P(Point(0,1,0), pitch);
    RotationMatrix Y(Point(0,0,1), yaw);

    auto m = R * P * Y;
    auto angles = get_roll_pitch_yaw(m);
    RotationMatrix T(angles);
    std::cout << "check .... ";
    if((m - T).l_inf_norm() < VAR_MLIB_ZERO_TOLERANCE)
      std::cout << "OK!" << std::endl;
    std::cout << "-------------------------" << std::endl;
  }
}
