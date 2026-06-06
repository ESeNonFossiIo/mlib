#include "../../test.h"

#include "numerix/math/spline.h"

#include <iomanip>
#include <iostream>

using namespace numerix;

// HermiteSpline<1> is not instantiated by any other test; this test also
// exercises HermiteSpline<3>::a() which hermite_03 does not call.
int main()
{
    print_title("HermiteSpline - dim 1 and dim 3 acceleration");

    std::cout << std::fixed << std::setprecision(5);

    // 1-D HermiteSpline
    Point p1_1d({0.0});
    Point p2_1d({1.0});
    Point v1_1d({1.0});
    Point v2_1d({1.0});
    HermiteSpline<1> hs1(p1_1d, p2_1d, v1_1d, v2_1d, 0.0, 1.0);
    std::cout << " hs1.p(0.0) = " << hs1.p(0.0).x() << std::endl;
    std::cout << " hs1.p(0.5) = " << hs1.p(0.5).x() << std::endl;
    std::cout << " hs1.v(0.5) = " << hs1.v(0.5).x() << std::endl;
    std::cout << " hs1.a(0.5) = " << hs1.a(0.5).x() << std::endl;

    // 3-D HermiteSpline a()
    Point p1_3d({0.0, 0.0, 0.0});
    Point p2_3d({1.0, 2.0, 3.0});
    Point v1_3d({1.0, 0.0, 0.0});
    Point v2_3d({0.0, 0.0, 1.0});
    HermiteSpline<3> hs3(p1_3d, p2_3d, v1_3d, v2_3d, 0.0, 1.0);
    Point acc = hs3.a(0.5);
    std::cout << " hs3.a(0.5).x = " << acc.x() << std::endl;
    std::cout << " hs3.a(0.5).y = " << acc.y() << std::endl;
    std::cout << " hs3.a(0.5).z = " << acc.z() << std::endl;

    return 0;
}
