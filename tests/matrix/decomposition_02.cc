#include "../test.h"

#include "numerix/math/matrix/decomposition.h"

#include <iostream>

using namespace numerix;

// Coverage entry point for SVD in builds without Eigen3 — the function only
// expands NUMERIX_UNUSED on its arguments and asserts(true), so the result
// matrices are intentionally not inspected here. The NUMERIX_USE_EIGEN3 branch
// is already covered by decomposition_00.
int main()
{
    print_title("SVD - no-op call");

    Matrixd M({{1, 2}, {3, 4}});
    Matrixd U, W, V;
    SVD(M, U, W, V);

    std::cout << "SVD returned" << std::endl;

    return 0;
}
