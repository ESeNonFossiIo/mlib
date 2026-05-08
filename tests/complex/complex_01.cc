#include "../test.h"

#include "mlib/math/complex.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title("Complex Numbers (extended)");

  // Default constructor
  Complex<int> c0;
  std::cout << " r0 = " << c0.r() << ", i0 = " << c0.i() << std::endl;

  // Single-argument constructor (real only)
  Complex<int> c1(7);
  std::cout << " r1 = " << c1.r() << ", i1 = " << c1.i() << std::endl;

  // Copy assignment operator
  Complex<int> a(2, 3);
  Complex<int> b;
  b = a;
  std::cout << " copy_assigned = " << b << std::endl;

  // Move assignment operator
  Complex<int> c;
  c = Complex<int>(4, -5);
  std::cout << " move_assigned = " << c << std::endl;

  // Compound assignment with another complex (operator*=)
  Complex<int> d(1, 2);
  Complex<int> e(3, 4);
  d *= e;
  std::cout << " compound      = " << d << std::endl;

  // Output operator with negative imaginary
  Complex<int> f(2, -7);
  std::cout << " neg_imag      = " << f << std::endl;

  return 0;
}
