#include "../test.h"

#include "mlib/math/matrix/utility.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title("Integral");

  {
    Matrixd m({{2,3},{1,1}});
    Point p({1,1});


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

  std::cout << " ------------------------------------------- " << std::endl;

  {

    std::size_t size = 5;

    Matrixd m(size,size);
    Point p;
    p.resize(size);

    for(size_t j = 0; j < size; ++j)
      {
        for(size_t i = 0; i < size-j; ++i)
          {
            m(i,j) = static_cast<double>(5-i-j);
          }
      }

    for(size_t j = 0; j < size; ++j)
      {
        p[j] = j;
      }

    auto a = gauss_elimination_upper(m,p);

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

    std::cout << " Compute gauss ........... ";
    if((m*a - p).l_2_norm() < VAR_MLIB_ZERO_TOLERANCE)
      std::cout << " [OK] " << std::endl;
    else
      std::cout << " [Error] " << std::endl;
  }
}
