#include "../test.h"

#include "numerix/math/point.h"

#include <iostream>

using namespace numerix;

int main()
{
    print_title("Point");

    {
        Point e1({1, 0, 0});
        std::cout << e1;
        e1.normalize();
        std::cout << e1;
    }

    {
        Point e1({1, 1, 0});
        std::cout << e1;
        e1.normalize();
        std::cout << e1;
    }

    {
        Point e1({1, 1, 1});
        std::cout << e1;
        e1.normalize();
        std::cout << e1;
    }
}
