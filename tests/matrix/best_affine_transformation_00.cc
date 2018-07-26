#include "../test.h"

#include "mlib/math/angle.h"
#include "mlib/math/constants.h"
#include "mlib/math/matrix/utility.h"

#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
  print_title("Roto-Translation Matrix");

  std::vector<Point> u, v;

  Matrixd A({{1,3,5},
    {2,4,2},
    {4,5,6}
  });
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
  double err = best_affine_transformation(u, v, M, c);
  std::cout << " error = " << err << std::endl;

  std::cout << M;
  std::cout << c;
}
