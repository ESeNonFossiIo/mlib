#include "../test.h"

#include "mlib/math/statistic.h"

#include <iostream>

using namespace mlib;

int main()
{
    print_title("Statistic Moments");

    std::vector<double> v = {1, 2, 3, 4, 5, 6};

    // n == 1 (uses early-return path with central=false)
    std::cout << " m1_raw       = " << moment(v, 1, false) << std::endl;

    // n == 1 with central=true (also uses early-return, but takes the central
    // branch first to compute mu, then returns mu)
    std::cout << " m1_central   = " << moment(v, 1, true) << std::endl;

    // Raw moment with n != 1: skips the central branch entirely (mu stays 0)
    // and computes (1/N) * sum(x^n).
    // For v = {1..6}, n=2: (1+4+9+16+25+36)/6 = 91/6 = 15.1667
    std::cout << " m2_raw       = " << moment(v, 2, false) << std::endl;

    // Third central moment of a symmetric vector is 0.
    std::cout << " m3_central   = " << moment(v, 3, true) << std::endl;

    return 0;
}
