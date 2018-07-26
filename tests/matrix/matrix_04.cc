#include "mlib/math/matrix/matrix.h"
#include "mlib/utility/string.h"

#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Matrix - cols and rows" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;


  Matrixd M({{4,3,3},{1,1,2}});
  std::cout << M << std::endl;
  std::cout << M.c(0) <<  std::endl;
  std::cout << M.c(1) <<  std::endl;
  std::cout << M.c(2) <<  std::endl;
  std::cout << M.r(0) <<  std::endl;
  std::cout << M.r(1) <<  std::endl;
}
