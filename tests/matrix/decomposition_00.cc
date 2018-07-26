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

  Matrixd M({{1,2,3},{2,3,9},{1,3,4}});

  Matrixd U,V,W;
  SVD(M,U,W,V);

  std::cout << std::fabs(U.det()) << std::endl;
  std::cout << std::fabs(V.det()) << std::endl;
  std::cout << ((M - U*W*V.t()).l_2_norm() < VAR_MLIB_ZERO_TOLERANCE ? "[OK]" :
                "[Fail]") << std::endl;
#else //_MYLIB_USE_EIGEN3
  make_test_pass("matrix/decomposition_00");
#endif //_MYLIB_USE_EIGEN3
}
