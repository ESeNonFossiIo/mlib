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
  std::cout << "  TEST for Matrix - inverse matrix" <<
            std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;


  Matrixd M({{4,3},{1,1}});
  std::cout << M.det() <<  std::endl;
  std::cout << M <<  std::endl;
  std::cout << M.inv() <<  std::endl;
  std::cout << M *M.inv() <<  std::endl;

}
