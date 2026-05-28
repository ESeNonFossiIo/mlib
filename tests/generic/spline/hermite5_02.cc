#include "../../test.h"

#include "mlib/math/spline.h"

#include <iomanip>
#include <iostream>

using namespace mlib;

// HermiteSpline5<1> and HermiteSpline5<2> are not instantiated by any other
// test; existing hermite5_* tests use HermiteSpline5<3>.
int main()
{
    print_title("HermiteSpline5 - dim 1 and dim 2");

    std::cout << std::fixed << std::setprecision(5);

    // 1-D
    Point p1_1d({0.0});
    Point p2_1d({1.0});
    Point v1_1d({1.0});
    Point v2_1d({1.0});
    Point a1_1d({0.0});
    Point a2_1d({0.0});
    HermiteSpline5<1> hs1(p1_1d, p2_1d, v1_1d, v2_1d, a1_1d, a2_1d, 0.0, 1.0);
    std::cout << " hs1.p(0.5) = " << hs1.p(0.5).x() << std::endl;
    std::cout << " hs1.v(0.5) = " << hs1.v(0.5).x() << std::endl;
    std::cout << " hs1.a(0.5) = " << hs1.a(0.5).x() << std::endl;

    // 2-D
    Point p1_2d({0.0, 0.0});
    Point p2_2d({1.0, 1.0});
    Point v1_2d({1.0, 0.0});
    Point v2_2d({0.0, 1.0});
    Point a1_2d({0.0, 0.0});
    Point a2_2d({0.0, 0.0});
    HermiteSpline5<2> hs2(p1_2d, p2_2d, v1_2d, v2_2d, a1_2d, a2_2d, 0.0, 1.0);
    std::cout << " hs2.p(0.5).x = " << hs2.p(0.5).x() << std::endl;
    std::cout << " hs2.p(0.5).y = " << hs2.p(0.5).y() << std::endl;
    std::cout << " hs2.v(0.5).x = " << hs2.v(0.5).x() << std::endl;
    std::cout << " hs2.a(0.5).y = " << hs2.a(0.5).y() << std::endl;

    return 0;
}
