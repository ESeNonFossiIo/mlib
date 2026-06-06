#include "../test.h"

#include "numerix/eigen/conversion.h"
#include <iostream>

using namespace numerix;

int main()
{

#ifdef NUMERIX_USE_EIGEN3
    print_title("Eigen Conversion from numerix to Eigen Matrix");

    Eigen::MatrixXd m(2, 3);
    m(0, 0) = 1;
    m(0, 1) = 2;
    m(0, 2) = 3;
    m(1, 0) = 4;
    m(1, 1) = 5;
    m(1, 2) = 6;
    std::cout << m << std::endl;
    std::cout << from_eigen_to_m_matrix(m) << std::endl;

#else  // NUMERIX_USE_EIGEN3
    make_test_pass("eigen/conversion_01");
#endif // NUMERIX_USE_EIGEN3
}
