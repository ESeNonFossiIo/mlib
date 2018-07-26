#include "../test.h"
#include "../test_compare.h"

#include "mlib/math/angle.h"
#include "mlib/math/constants.h"
#include "mlib/math/matrix/utility.h"

#include "mlib/math/matrix/rotation.h"

#include <iostream>
#include <vector>

using namespace _mlib;

int main()
{
#ifdef _MYLIB_USE_EIGEN3
  print_title("Roto-Translation Matrix");

  std::vector<Point> u, v;

  Angle roll(0, AngleType::rad);
  Angle pitch(1.3, AngleType::rad);
  Angle yaw(2.0, AngleType::rad);

  TaitBryanAngles angles(roll, pitch, yaw);

  auto A = RotationMatrix(angles);

  Point b({1,3,2});

  std::cout << " A det = " << A.det() << std::endl;

  u.push_back(Point(0.0,0.0,1.0));
  u.push_back(Point(0.0,1.0,0.0));
  u.push_back(Point(1.0,0.0,0.0));
  u.push_back(Point(1.0,1.0,0.0));
  u.push_back(Point(2.0,0.0,0.0));
  u.push_back(Point(2.0,1.0,0.0));

  for(auto p : u)
    {
      v.push_back(A*p + b);
    }


  Matrix<double> M;
  Point c;
  double err = rototranslation_matrix(u, v, M, c);

  std::cout << " Error .......... " << are_equal(err) << std::endl;
  std::cout << " Matrix ......... "<< are_equal((M-A).l_2_norm()) << std::endl;
  std::cout << " Vector ......... "<< are_equal((b-c).l_2_norm()) << std::endl;

#else //_MYLIB_USE_EIGEN3
  make_test_pass("matrix/rototranslation_matrix_03");
#endif //_MYLIB_USE_EIGEN3
}
