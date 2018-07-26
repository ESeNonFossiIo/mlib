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
            "  TEST for Matrix - det, inverse, and cofactor" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;


  Matrixd M({{1,1,1},{0,2,0},{0,0,1}});
  std::cout << M.det() <<  std::endl;
  std::cout << M <<  std::endl;
  std::cout << M.inv() <<  std::endl;
  std::cout << M *M.inv() <<  std::endl;

  std::cout << M.cofactor(1,2) <<  std::endl;
  std::cout << M.cofactor(1,1) <<  std::endl;
  std::cout << M.cofactor(2,1) <<  std::endl;
  std::cout << M.cofactor(2,2) <<  std::endl;
}
