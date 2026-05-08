#include "../test.h"

#include "mlib/math/integral.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title("Integral (extended)");

  // TrapezoidalRule on x: integral from 0 to 1 of x dx = 0.5
  {
    TrapezoidalRule tr;
    Integral I(tr, 0.001);

    const std::function<double(double)>& f = [](double x)
    {
      return x;
    };

    std::cout << I(f, 1.0) << std::endl;
  }

  // TrapezoidalRule on x*x: integral from 0 to 1 of x^2 dx = 0.333333
  {
    TrapezoidalRule tr;
    Integral I(tr, 0.001);

    const std::function<double(double)>& f = [](double x)
    {
      return x*x;
    };

    std::cout << I(f, 1.0) << std::endl;
  }

  // Negative-x branch: MidpointMethod, integral from -1 to 0 of x dx = -0.5
  {
    MidpointMethod mm;
    Integral I(mm, 0.001);

    const std::function<double(double)>& f = [](double x)
    {
      return x;
    };

    std::cout << I(f, -1.0) << std::endl;
  }

  // Exercise Quadrature and Integral copy constructors
  {
    MidpointMethod mm;
    Quadrature qcopy(mm);
    Integral I(mm, 0.001);
    Integral Icopy(I);

    const std::function<double(double)>& f = [](double x)
    {
      return x*x;
    };

    std::cout << Icopy(f, 1.0) << std::endl;
  }

  return 0;
}
