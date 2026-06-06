#include "numerix/math/matrix/matrix.h"

#include <iostream>
#include <vector>

using namespace numerix;

int main()
{
    std::cout << "=================================================" << std::endl;
    std::cout << "  TEST for Matrix" << std::endl;
    std::cout << "=================================================" << std::endl;

    for (size_t i = 1; i < 5; ++i) {
        IdentityMatrix m(i);
        std::cout << m;
    }
}
