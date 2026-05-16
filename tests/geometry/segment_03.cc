#include "../test.h"

#include "mlib/math/geometry/segment.h"

using namespace mlib;

// Coverage helper for Segment scalar operators, get_length() and
// min_distance().  The numeric outputs are deterministic.
int main()
{
    print_title("Segment - 03");

    Point a(0, 0);
    Point b(3, 4);
    Segment ab(a, b);

    std::cout << " length      = " << ab.get_length() << std::endl;

    Segment scaled = ab * 2.0;
    std::cout << " scaled.len  = " << scaled.get_length() << std::endl;
    std::cout << " scaled.p1   = " << scaled.get_extreme_points().first << std::endl;
    std::cout << " scaled.p2   = " << scaled.get_extreme_points().second << std::endl;
    std::cout << " (ab*3).len  = " << (ab * 3.0).get_length() << std::endl;

    Segment ab_copy = ab;
    Segment shrunk = ab_copy / 2.0;
    std::cout << " shrunk.len  = " << shrunk.get_length() << std::endl;
    std::cout << " shrunk.p1   = " << shrunk.get_extreme_points().first << std::endl;
    std::cout << " shrunk.p2   = " << shrunk.get_extreme_points().second << std::endl;
    std::cout << " (ab_copy/3) = " << (ab_copy / 3.0).get_length() << std::endl;

    Segment inplace = ab;
    inplace *= 0.5;
    std::cout << " inplace*=   = " << inplace.get_length() << std::endl;

    Segment inplace_div = ab;
    inplace_div /= 2.0;
    std::cout << " inplace/=   = " << inplace_div.get_length() << std::endl;

    // Two parallel non-overlapping segments — min_distance picks endpoints.
    Point c(0, 10);
    Point d(3, 14);
    Segment cd(c, d);
    std::cout << " min_dist    = " << min_distance(ab, cd) << std::endl;

    return 0;
}
