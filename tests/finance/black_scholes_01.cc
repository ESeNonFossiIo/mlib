#include "../test.h"

#include <mlib/finance/black_scholes.h>
#include "mlib/math/statistics/norm_dist.h"

#include <iomanip>
#include <iostream>

using namespace mlib;
using namespace mlib::finance;
using namespace mlib::math::statistics;

int main()
{
    print_title("Black-Scholes - extra branches");

    std::cout << std::fixed << std::setprecision(5);

    // norm_pdf — never called by existing tests
    std::cout << " norm_pdf(0)   = " << norm_pdf(0.0) << std::endl;
    std::cout << " norm_pdf(1)   = " << norm_pdf(1.0) << std::endl;

    // BSPrice() with OptionType::None → else branch, return -1
    double p = BSPrice(100.0, 100.0, 0.05, 0.2, 1.0, OptionType::None);
    std::cout << " price_none    = " << p << std::endl;

    return 0;
}
