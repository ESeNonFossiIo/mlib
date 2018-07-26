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

  {
    Point u({1,0,0});
    Angle   theta1(180, AngleType::deg);
    Angle   theta2(M_PI, AngleType::rad);
    auto matrix1 = RotationMatrix(u, theta1);
    auto matrix2 = RotationMatrix(u, theta2);
    std::cout << matrix1;
    std::cout << matrix2;
    std::cout << matrix1 - matrix2;
    std::cout << matrix1 * matrix2;
  }
  {
    Point u({0,1,0});
    Angle   theta1(180, AngleType::deg);
    Angle   theta2(M_PI, AngleType::rad);
    auto matrix1 = RotationMatrix(u, theta1);
    auto matrix2 = RotationMatrix(u, theta2);
    std::cout << matrix1;
    std::cout << matrix2;
    std::cout << matrix1 - matrix2;
    std::cout << matrix1 * matrix2;
  }
  {
    Point u({0,0,1});
    Angle   theta1(180, AngleType::deg);
    Angle   theta2(M_PI, AngleType::rad);
    auto matrix1 = RotationMatrix(u, theta1);
    auto matrix2 = RotationMatrix(u, theta2);
    std::cout << matrix1;
    std::cout << matrix2;
    std::cout << matrix1 - matrix2;
    std::cout << matrix1 * matrix2;
  }
  {
    Point u1({1,0,0});
    Point u2({-1,0,0});
    Angle   theta1(180,  AngleType::deg);
    Angle   theta2(M_PI, AngleType::rad);
    auto matrix1 = RotationMatrix(u1, theta1);
    auto matrix2 = RotationMatrix(u2, theta2);
    std::cout << matrix1;
    std::cout << matrix2;
    std::cout << matrix1 - matrix2;
    std::cout << matrix1 * matrix2;
  }
}
