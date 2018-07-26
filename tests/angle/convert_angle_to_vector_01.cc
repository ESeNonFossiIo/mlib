#include "mlib/math/angle.h"
#include <iostream>

// per std::setw
#include <iomanip>

using namespace _mlib;

void test(const double& angle,
          const double& x,
          const double& y,
          const double& xaxis,
          const double& yaxis,
          const bool& clockwise)
{
  Angle a(angle, AngleType::deg);
  Point q(x, y);
  Point p(xaxis, yaxis);
  Point r = convert_angle_to_vector(convert_vector_to_angle(q, p, clockwise), p,
                                    clockwise);
  Angle b   = convert_vector_to_angle(convert_angle_to_vector(a, p, clockwise), p,
                                      clockwise);

  if((q-r).l_2_norm() < VAR_MLIB_ZERO_TOLERANCE)
    std::cout << "[OK]";
  else
    std::cout << "[Fail]";
  std::cout <<  " - ";
  if(std::fmod(a.deg() - b.deg(), 360.0) < VAR_MLIB_ZERO_TOLERANCE)
    std::cout << "[OK]";
  else
    std::cout << "[Fail]";
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

// Test remove_singularities function:
////////////////////////////////////////////////////////////////////////////////
  for(int i = -360; i<=360; i+=30)
    {
      std::cout << " i  = " << std::setw(4) << i << " -----> ";
      test(i * 1.0, 0.0, 1.0,  1.0, 0.0, true);
      std::cout <<  " - ";
      test(i * 1.0, 0.0, 1.0,  1.0, 0.0, false);
      std::cout << std::endl;
    }

  return 0;
}
