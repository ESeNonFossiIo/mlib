#include "../test.h"

#include "mlib/math/complex.h"

using namespace mlib;

int main()
{
  print_title("Complex Numbers");

  {
    Complex<int> c1(1,1);
    Complex<int> c2(1,-1);

    std::cout << c1 << std::endl;
    std::cout << c2 << std::endl;

    std::cout << (c1*c2) << std::endl;
    std::cout << (c2*c1) << std::endl;
    std::cout << (2*c1) << std::endl;
  }

}
