#include "../test.h"

#include "mlib/math/matrix/utility.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title("Integral");

  {
    Point P1({1,2});
    Point P2({0,1});
    Point P3({1,1});
    Point Q1({1,2});
    Point Q2({0,1});
    Point Q3({1,1});

    Matrixd m({ {P1[0], P1[1],0, 0,  1,  0},
      {0, 0, P1[0], P1[1],  0,  1},
      {P2[0], P2[1], 0, 0,  1,  0},
      {0, 0, P2[0], P2[1],  0,  1},
      {P3[0], P3[1], 0, 0,  1,  0},
      {0, 0, P3[0], P3[1],  0,  1}
    });

    Point p({Q1[0], Q1[1], Q2[0], Q2[1], Q3[0], Q3[1]});

    std::cout << " Compute upper triangular system " << std::endl;
    auto T = upper_triangular(m,p);

    std::cout << " Solve upper triangular system " << std::endl;
    auto s = solve_upper_triangular(T.first, T.second);

    std::cout << " Solve output ............ ";
    if((T.first*s - T.second).l_2_norm() < VAR_MLIB_ZERO_TOLERANCE)
      std::cout << " [OK] " << std::endl;
    else
      std::cout << " [Error] " << std::endl;

    std::cout << " Compute output .......... ";
    if((m*s - p).l_2_norm() < VAR_MLIB_ZERO_TOLERANCE)
      std::cout << " [OK] " << std::endl;
    else
      std::cout << " [Error] " << std::endl;
  }

}
