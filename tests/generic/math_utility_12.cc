#include "../test.h"

#include "mlib/math/utility.h"

#include <iomanip>
#include <iostream>
#include <vector>

using namespace mlib;

// Instantiate sgn<float>, truncate<float>, truncate_vec<float> — the existing
// math_utility_* tests only exercise the double variants.
int main()
{
    print_title("Math utility - float instantiations");

    std::cout << std::fixed << std::setprecision(3);

    std::cout << " sgn(2.5f)  = " << sgn<float>(2.5f) << std::endl;
    std::cout << " sgn(-1.5f) = " << sgn<float>(-1.5f) << std::endl;
    std::cout << " sgn(0.0f)  = " << sgn<float>(0.0f) << std::endl;

    std::cout << " trunc(5.0f, 0.0f, 3.0f) = " << truncate<float>(5.0f, 0.0f, 3.0f) << std::endl;
    std::cout << " trunc(-2.0f, 0.0f, 3.0f) = " << truncate<float>(-2.0f, 0.0f, 3.0f) << std::endl;
    std::cout << " trunc(1.5f, 0.0f, 3.0f) = " << truncate<float>(1.5f, 0.0f, 3.0f) << std::endl;

    std::vector<float> v = {-2.0f, 0.0f, 1.5f, 5.0f, 10.0f};
    std::vector<float> r = truncate_vec<float>(v, 0.0f, 3.0f);
    for (std::size_t i = 0; i < r.size(); ++i)
        std::cout << " r[" << i << "] = " << r[i] << std::endl;

    return 0;
}
