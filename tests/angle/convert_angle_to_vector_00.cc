#include "mlib/math/angle.h"
#include <iostream>

// per std::setw
#include <iomanip>

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

// Test remove_singularities function:
////////////////////////////////////////////////////////////////////////////////
  {
    Angle a(45.0, AngleType::deg);
    Point p(1.0, 0.0);
    Point q = convert_angle_to_vector(a, p, true);
    std::cout << p;
    std::cout << convert_angle_to_vector(a, p, true);
    std::cout << convert_angle_to_vector(a, q, false);
  }
  {
    Angle a(30.0, AngleType::deg);
    Point p(0.0, 1.0);
    Point q = convert_angle_to_vector(a, p, true);
    std::cout << p;
    std::cout << convert_angle_to_vector(a, p, true);
    std::cout << convert_angle_to_vector(a, q, false);
  }
  return 0;
}
