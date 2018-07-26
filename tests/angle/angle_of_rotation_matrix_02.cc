#include "mlib/math/matrix/rotation.h"
#include "mlib/math/constants.h"

#include <iostream>
#include <vector>
#include <cmath>

using namespace mlib;

void test_1(double roll_angle,
            double pitch_angle,
            double yaw_angle)
{
  Angle roll(roll_angle);
  Angle pitch(pitch_angle);
  Angle yaw(yaw_angle);

  RotationMatrix R(Point(1,0,0), roll);
  RotationMatrix P(Point(0,1,0), pitch);
  RotationMatrix Y(Point(0,0,1), yaw);

  // auto m = Y * P * R;
  // auto angles = get_roll_pitch_yaw(m, RotationType::ZYX);
  //
  auto m = R * P * Y;
  auto angles = get_roll_pitch_yaw(m,RotationType::XYZ,M_PI);

  std::cout << "check .... ";

  if(std::abs(roll.rad() - angles.roll()) > VAR_MLIB_ZERO_TOLERANCE)
    {
      std::cout << " roll  -> "<< roll.rad() << std::endl;
      std::cout << "          "<< angles.roll() << std::endl;
      std::cout << "          "<< roll.rad() - angles.roll() << std::endl;
    }
  else if(std::abs(pitch.rad() - angles.pitch()) > VAR_MLIB_ZERO_TOLERANCE)
    {
      std::cout << " pitch -> "<< pitch.rad() << std::endl;
      std::cout << "          "<< angles.pitch() << std::endl;
      std::cout << "          "<< pitch.rad() - angles.pitch() << std::endl;
    }
  else if(std::abs(yaw.rad() - angles.yaw()) > VAR_MLIB_ZERO_TOLERANCE)
    {
      std::cout << "   yaw -> "<< yaw.rad() << std::endl;
      std::cout << "          "<< angles.yaw() << std::endl;
      std::cout << "          "<< yaw.rad() - angles.yaw() << std::endl;
    }
  else
    {
      std::cout << " OK " << std::endl;
    }

}

void test_2(double roll_angle,
            double pitch_angle,
            double yaw_angle)
{
  Angle roll(roll_angle);
  Angle pitch(pitch_angle);
  Angle yaw(yaw_angle);

  RotationMatrix R(Point(1,0,0), roll);
  RotationMatrix P(Point(0,1,0), pitch);
  RotationMatrix Y(Point(0,0,1), yaw);

  auto m = Y * P * R;
  auto angles = get_roll_pitch_yaw(m, RotationType::ZYX);

  // auto m = R * P * Y;
  // auto angles = get_roll_pitch_yaw(m,RotationType::XYZ,M_PI);

  std::cout << "check .... ";

  if(std::abs(roll.rad() - angles.roll()) > VAR_MLIB_ZERO_TOLERANCE)
    {
      std::cout << " roll  -> "<< roll.rad() << std::endl;
      std::cout << "          "<< angles.roll() << std::endl;
      std::cout << "          "<< roll.rad() - angles.roll() << std::endl;
    }
  else if(std::abs(pitch.rad() - angles.pitch()) > VAR_MLIB_ZERO_TOLERANCE)
    {
      std::cout << " pitch -> "<< pitch.rad() << std::endl;
      std::cout << "          "<< angles.pitch() << std::endl;
      std::cout << "          "<< pitch.rad() - angles.pitch() << std::endl;
    }
  else if(std::abs(yaw.rad() - angles.yaw()) > VAR_MLIB_ZERO_TOLERANCE)
    {
      std::cout << "   yaw -> "<< yaw.rad() << std::endl;
      std::cout << "          "<< angles.yaw() << std::endl;
      std::cout << "          "<< yaw.rad() - angles.yaw() << std::endl;
    }
  else
    {
      std::cout << " OK " << std::endl;
    }

}

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Angle" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  int size = 10;
  for(int i = -(size - 1); i<=(size - 1); ++i)
    for(int j = -(size/2 - 1); j<=(size/2 - 1); ++j)
      for(int k = -(size - 1); k<=(size - 1); ++k)
        {
          // std::cout << " ======================= " << std::endl;
          // std::cout << " > roll =  " << i * M_PI/size << std::endl;
          // std::cout << " > pitch = " << j * M_PI/size << std::endl;
          // std::cout << " > yaw =   " << k * M_PI/size << std::endl;
          test_1(i * M_PI/size,j*M_PI/size,k*M_PI/size);
          test_2(i * M_PI/size,j*M_PI/size,k*M_PI/size);
        }

  return 0;
}
