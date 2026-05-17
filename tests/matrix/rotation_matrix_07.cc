#include "../test.h"

#include "mlib/math/angle.h"
#include "mlib/math/matrix/rotation.h"

#include <iomanip>
#include <iostream>

using namespace mlib;

int main()
{
    print_title("RotationMatrix - Matrix axis constructor");

    std::cout << std::fixed << std::setprecision(5);

    // RotationMatrix(Matrix axis_, Angle theta_) — covers lines 58-69 of rotation.cc
    Matrixd axis_m({{0.0}, {0.0}, {1.0}});
    Angle theta(M_PI / 4.0);
    RotationMatrix R(axis_m, theta);

    std::cout << " R(0,0) = " << R(0, 0) << std::endl;
    std::cout << " R(1,1) = " << R(1, 1) << std::endl;
    std::cout << " R(0,1) = " << R(0, 1) << std::endl;

    return 0;
}
