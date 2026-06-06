#include "numerix/math/angle.h"
#include "numerix/math/constants.h"
#include "numerix/math/matrix/rotation.h"

#include <iostream>
#include <vector>

using namespace numerix;

int main()
{
    std::cout << "=================================================" << std::endl;
    std::cout << "  TEST for Matrix" << std::endl;
    std::cout << "=================================================" << std::endl;

    Point u({1, 0, 0});
    Angle theta(M_PI, AngleType::rad);
    auto matrix =
        RotationMatrix(u,
                       theta); // std::sin(theta)*M + std::cos(theta)*(id - u*u.t()) + u*u.t();
    std::cout << matrix;
}
