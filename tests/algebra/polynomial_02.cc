#include "../test.h"

#include "mlib/math/algebra.h"

#include <iostream>
#include <vector>

using namespace mlib;

int main()
{
  print_title("Polynomial (extended)");

  // Default constructor: zero polynomial of degree 0
  {
    Polynomial p;
    std::cout << " default      = " << p << std::endl;
    std::cout << " default_deg  = " << p.deg() << std::endl;
    std::cout << " default_size = " << p.size() << std::endl;
  }

  // Vector constructor
  {
    std::vector<double> coeffs = {2, 0, 1};   // 2 + x^2
    Polynomial q(coeffs);
    std::cout << " vec_ctor     = " << q << std::endl;

    // operator() evaluation: q(0) = 2, q(1) = 3, q(2) = 6
    std::cout << " q_at_0       = " << q(0.0) << std::endl;
    std::cout << " q_at_1       = " << q(1.0) << std::endl;
    std::cout << " q_at_2       = " << q(2.0) << std::endl;
  }

  // Non-const operator[] (mutates)
  {
    Polynomial r({1, 2, 3});
    r[1] = 5;                  // change middle coefficient
    std::cout << " mutated      = " << r << std::endl;

    // Const access via const reference
    const Polynomial& cr = r;
    std::cout << " const_at_2   = " << cr[2] << std::endl;
  }

  // d(0) returns *this; d(i) where i > deg returns zero polynomial
  {
    Polynomial p({1, 2, 1});
    std::cout << " d0           = " << p.d(0) << std::endl;
    std::cout << " d10          = " << p.d(10) << std::endl;
    std::cout << " d10_deg      = " << p.d(10).deg() << std::endl;
  }

  return 0;
}
