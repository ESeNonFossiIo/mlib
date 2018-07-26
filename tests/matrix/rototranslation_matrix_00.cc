#include "../test.h"

#include "mlib/math/angle.h"
#include "mlib/math/constants.h"
#include "mlib/math/matrix/utility.h"

#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
#ifdef MLIB_USE_EIGEN3
  print_title("Roto-Translation Matrix");

  std::vector<Point> u, v;


  u.push_back(Point(1.0,0.0,0.0));
  u.push_back(Point(0.0,1.0,0.0));
  u.push_back(Point(0.0,0.0,1.0));
  u.push_back(Point(0.0,0.0,0.0));


  v.push_back(Point(1.0,0.0,0.0));
  v.push_back(Point(0.0,1.0,0.0));
  v.push_back(Point(0.0,0.0,1.0));
  v.push_back(Point(0.0,0.0,0.0));

  Matrix<double> M;
  Point b;
  std::cout << " error = " << rototranslation_matrix(u, v, M, b) << std::endl;

  std::cout << M;
  std::cout << b;

#else //MLIB_USE_EIGEN3
  make_test_pass("matrix/rototranslation_matrix_00");
#endif //MLIB_USE_EIGEN3
}
