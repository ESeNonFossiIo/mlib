#include "../test.h"

#include "mlib/math/matrix/decomposition.h"

#include <cmath>
#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
    // QR is pure mlib code and does not require Eigen3, so we test it
    // unconditionally to keep the function covered in the default build.
    print_title("SVD - Decomposition");

    Matrixd M({{12, -51, 4}, {6, 167, -68}, {-4, 24, -41}});

    Matrixd Q, R;
    QR(M, Q, R);

    std::cout << Q << std::endl;
    std::cout << R << std::endl;
    std::cout << ((M - Q * R).l_2_norm() < VAR_MLIB_ZERO_TOLERANCE ? "[OK]" : "[Fail]")
              << std::endl;
}
