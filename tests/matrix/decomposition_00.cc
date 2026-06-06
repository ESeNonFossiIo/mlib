#include "../test.h"

#include "numerix/math/matrix/decomposition.h"

#include <cmath>
#include <iostream>
#include <vector>

using namespace numerix;

int main()
{
#ifdef NUMERIX_USE_EIGEN3
    print_title("SVD - Decomposition");

    Matrixd M({{1, 2, 3}, {2, 3, 9}, {1, 3, 4}});

    Matrixd U, V, W;
    SVD(M, U, W, V);

    std::cout << std::fabs(U.det()) << std::endl;
    std::cout << std::fabs(V.det()) << std::endl;
    std::cout << ((M - U * W * V.t()).l_2_norm() < VAR_NUMERIX_ZERO_TOLERANCE ? "[OK]" : "[Fail]")
              << std::endl;
#else  // NUMERIX_USE_EIGEN3
    make_test_pass("matrix/decomposition_00");
#endif // NUMERIX_USE_EIGEN3
}
