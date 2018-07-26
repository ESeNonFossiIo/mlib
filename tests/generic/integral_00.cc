#include "../test.h"

#include "mlib/math/integral.h"

#include <iostream>

using namespace _mlib;

int main()
{
  print_title("Integral");

  {
    MidpointMethod mm;
    Integral I(mm, 0.001);

    const std::function<double(double)>& f = [](double x)
    {
      return x;
    };

    std::cout << I(f, 1.0) << std::endl;
  }

  {
    MidpointMethod mm;
    Integral I(mm, 0.001);

    const std::function<double(double)>& f = [](double x)
    {
      return x*x;
    };

    std::cout << I(f, 1.0) << std::endl;
  }

  {
    MidpointMethod mm;
    Integral I(mm, 0.001);

    const std::function<double(double)>& f = [](double x)
    {
      return x*x*x;
    };

    std::cout << I(f, 1.0) << std::endl;
  }
}
