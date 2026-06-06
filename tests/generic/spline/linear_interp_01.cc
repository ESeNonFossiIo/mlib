#include "../../test.h"

#include "numerix/math/spline.h"

#include <iomanip>
#include <iostream>

using namespace numerix;

// Instantiates LinearInterpolation<1> and LinearInterpolation<3> (and the
// underlying BaseInterpolation<1>/<3>), which are not otherwise exercised.
int main()
{
    print_title("LinearInterpolation - dim 1 and dim 3");

    std::cout << std::fixed << std::setprecision(5);

    // 1-D
    Point p1_1d({0.0});
    Point p2_1d({2.0});
    LinearInterpolation<1> li1(p1_1d, p2_1d, 0.0, 1.0);
    std::cout << " li1(0.0) = " << li1(0.0).x() << std::endl;
    std::cout << " li1(0.5) = " << li1(0.5).x() << std::endl;
    std::cout << " li1(1.0) = " << li1(1.0).x() << std::endl;

    // 3-D
    Point p1_3d({0.0, 0.0, 0.0});
    Point p2_3d({1.0, 2.0, 3.0});
    LinearInterpolation<3> li3(p1_3d, p2_3d, 0.0, 1.0);
    std::cout << " li3(0.5).x = " << li3(0.5).x() << std::endl;
    std::cout << " li3(0.5).y = " << li3(0.5).y() << std::endl;
    std::cout << " li3(0.5).z = " << li3(0.5).z() << std::endl;

    // Exercise get_normalized_time on the underlying BaseInterpolation.
    // (Uses the dim=1 instance since the method exists on every dim.)
    std::cout << " nt(0.25)   = " << li1.get_normalized_time(0.25) << std::endl;

    return 0;
}
