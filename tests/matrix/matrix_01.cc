#include "mlib/math/matrix/matrix.h"
#include "mlib/utility/string.h"

#include <iostream>
#include <vector>

using namespace _mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout <<
            "  TEST for Matrix - l infinity norm and l2 norm" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;


  Matrixd M({{1,0,1},{0,1,0},{0,0,1}});
  std::cout << M << std::endl;
  std::cout << M.l_inf_norm()
            << " - " << M.l_1_norm()
            << " - " << M.l_p_norm(1)
            << " - " << M.l_2_norm()
            << " - " << M.l_p_norm(2)
            <<
            std::endl;
  std::cout << hline(40, "#") << std::endl;

  M(0,0) = -2;
  std::cout << M << std::endl;
  std::cout << M.l_inf_norm()
            << " - " << M.l_1_norm()
            << " - " << M.l_p_norm(1)
            << " - " << M.l_2_norm()
            << " - " << M.l_p_norm(2)
            <<
            std::endl;
  std::cout << hline(40, "#") << std::endl;

  M *= -10;
  std::cout << M << std::endl;
  std::cout << M.l_inf_norm()
            << " - " << M.l_1_norm()
            << " - " << M.l_p_norm(1)
            << " - " << M.l_2_norm()
            << " - " << M.l_p_norm(2)
            <<
            std::endl;
  std::cout << hline(40, "#") << std::endl;

  M /= 200;
  std::cout << M << std::endl;
  std::cout << M.l_inf_norm()
            << " - " << M.l_1_norm()
            << " - " << M.l_p_norm(1)
            << " - " << M.l_2_norm()
            << " - " << M.l_p_norm(2)
            <<
            std::endl;
  std::cout << hline(40, "#") << std::endl;

  M(0,0) = 0.00002;
  std::cout << M << std::endl;
  std::cout << M.l_inf_norm()
            << " - " << M.l_1_norm()
            << " - " << M.l_p_norm(1)
            << " - " << M.l_2_norm()
            << " - " << M.l_p_norm(2)
            <<
            std::endl;
}
