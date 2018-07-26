#include "../test.h"
#include "../test_compare.h"

#include "mlib/math/matrix/rotation.h"

#include <iostream>
#include <vector>

using namespace mlib;


int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Rodriguez RotationMatrix" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  Point rodriguez(
    0.8229077413424785,
    -1.01946248418652,
    1.4187621673937);

  Matrixd M(
  {
    {  -0.107,-0.99175,-0.070548},
    { 0.38321,0.024336,-0.92334},
    { 0.91744,-0.12584,0.37745}
  });

  RotationMatrix R(rodriguez);

  std::cout << ((R-M).l_2_norm() < 1e-5) <<  std::endl;
}
