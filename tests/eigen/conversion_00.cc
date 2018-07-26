#include "../test.h"

#include "mlib/eigen/conversion.h"
#include <iostream>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_EIGEN3
  print_title("Eigen Conversion from Eigen to mlib Matrix");

  Matrixd m({{1,2,3},{4,5,6}});
  std::cout << m << std::endl;
  std::cout << from_m_to_eigen_matrix(m) << std::endl;

#else //MLIB_USE_EIGEN3
  make_test_pass("eigen/conversion_00");
#endif //MLIB_USE_EIGEN3

}
