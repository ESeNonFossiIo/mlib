#include "../../test.h"

#include "mlib/math/spline.h"

#include <iomanip>
#include <iostream>

using namespace mlib;

int main()
{
    print_title("HermiteSpline - acceleration (a)");

    std::cout << std::fixed << std::setprecision(5);

    Point p1({0.0, 0.0});
    Point p2({1.0, 0.0});
    Point v1({1.0, 0.0});
    Point v2({1.0, 0.0});

    HermiteSpline<2> hs(p1, p2, v1, v2, 0.0, 1.0);

    // Call a() — covers lines 102-115 of spline.cc
    Point acc = hs.a(0.0);
    std::cout << " a(0).x = " << acc.x() << std::endl;
    std::cout << " a(0).y = " << acc.y() << std::endl;

    Point acc1 = hs.a(1.0);
    std::cout << " a(1).x = " << acc1.x() << std::endl;

    return 0;
}
