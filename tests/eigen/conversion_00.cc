#include "../test.h"

#include "numerix/eigen/conversion.h"
#include <iostream>

using namespace numerix;

int main()
{
#ifdef NUMERIX_USE_EIGEN3
    print_title("Eigen Conversion from Eigen to numerix Matrix");

    Matrixd m({{1, 2, 3}, {4, 5, 6}});
    std::cout << m << std::endl;
    std::cout << from_m_to_eigen_matrix(m) << std::endl;

#else  // NUMERIX_USE_EIGEN3
    make_test_pass("eigen/conversion_00");
#endif // NUMERIX_USE_EIGEN3
}
