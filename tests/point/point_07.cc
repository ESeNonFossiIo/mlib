#include "../test.h"

#include "numerix/math/point.h"

#include <iomanip>
#include <iostream>

using namespace numerix;

int main()
{
    print_title("Point - uncovered paths");

    std::cout << std::fixed << std::setprecision(3);

    // Point(bool normalize=true) — normalized single-element constructor
    Point pn(true);
    std::cout << " pn.dim = " << pn.dim() << std::endl;

    // Point(initializer_list, normalize=true) — normalizes {3,4} to {0.6,0.8}
    Point pl({3.0, 4.0}, true);
    std::cout << " pl.x  = " << pl.x() << std::endl;
    std::cout << " pl.y  = " << pl.y() << std::endl;

    // Point(x, y, z, t) — 4-arg constructor
    Point p4(1.0, 2.0, 3.0, 4.0);
    std::cout << " p4.x  = " << p4.x() << std::endl;
    std::cout << " p4.w  = " << p4.w() << std::endl;

    // non-const operator()(size_t i) — single-index write
    Point q({1.0, 2.0, 3.0});
    q(0) = 5.0;
    std::cout << " q(0)  = " << q(0) << std::endl;

    // const operator()(size_t i) — single-index read
    const Point& cq = q;
    std::cout << " cq(1) = " << cq(1) << std::endl;

    // vector_product with 2D points — cross product
    Point a({1.0, 0.0});
    Point b({0.0, 1.0});
    std::cout << " vprod = " << vector_product(a, b) << std::endl;

    return 0;
}
