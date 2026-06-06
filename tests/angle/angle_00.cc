#include "numerix/math/angle.h"
#include "numerix/math/constants.h"

#include <iostream>
#include <vector>

using namespace numerix;

int main()
{
    std::cout << "=================================================" << std::endl;
    std::cout << "  TEST for Angle" << std::endl;
    std::cout << "=================================================" << std::endl;

    std::vector<double> angle = {0, 30, 45, 90, 180};

    for (auto it = angle.begin(); it != angle.end(); ++it) {
        Angle angle(*it, AngleType::deg);
        std::cout << "deg = " << angle.deg() << std::endl << "rad = " << angle.rad() << std::endl;
    }
}
