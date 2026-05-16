#include "../test.h"

#include "mlib/math/geometry/clothoid.h"

#include <iomanip>
#include <iostream>

using namespace mlib;

// Hits Clothoid::l, Clothoid::k and the ApproximatedClothoid counterparts
// which the other clothoid_* tests do not call.
int main()
{
    print_title("Clothoid - l/k accessors");

    std::cout << std::fixed << std::setprecision(3);

    Clothoid c(2.0);
    std::cout << " c.l(3)  = " << c.l(3.0) << std::endl;
    std::cout << " c.k(6)  = " << c.k(6.0) << std::endl;

    ApproximatedClothoid ac(2.0);
    std::cout << " ac.l(3) = " << ac.l(3.0) << std::endl;
    std::cout << " ac.k(6) = " << ac.k(6.0) << std::endl;

    return 0;
}
