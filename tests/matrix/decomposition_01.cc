#include "../test.h"

#include "mlib/math/matrix/decomposition.h"

#include <iostream>
#include <vector>
#include <cmath>

using namespace _mlib;

int main()
{
#ifdef _MYLIB_USE_EIGEN3
  print_title("SVD - Decomposition");

  Matrixd M({{12,-51,4},{6,167,-68},{-4,24,-41}});

  Matrixd Q, R;
  QR(M,Q,R);

  std::cout << Q << std::endl;
  std::cout << R << std::endl;
  std::cout << ((M - Q*R).l_2_norm() < VAR_MLIB_ZERO_TOLERANCE ? "[OK]" :
                "[Fail]") << std::endl;
#else //_MYLIB_USE_EIGEN3
  make_test_pass("matrix/decomposition_00");
#endif //_MYLIB_USE_EIGEN3
}
