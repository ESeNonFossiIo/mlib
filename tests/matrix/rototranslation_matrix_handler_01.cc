#include "../test.h"
#include "../test_compare.h"

#include "mlib/math/angle.h"
#include "mlib/math/matrix/rotation.h"
#include "mlib/math/matrix/utility.h"

#include <iostream>
#include <vector>

using namespace mlib;

// Coverage for the RotoTranslationMatrixHandler getters that the existing
// handler_00 test does not touch: update_mask, get_error_on_point,
// get_volume_diff, get_number_of_points.
int main()
{
#ifdef MLIB_USE_EIGEN3
    print_title("RotoTranslationMatrixHandler - mask + getters");

    Angle roll(0.5, AngleType::rad);
    Angle pitch(0.2, AngleType::rad);
    Angle yaw(0.4, AngleType::rad);

    TaitBryanAngles angles(roll, pitch, yaw);
    auto A = RotationMatrix(angles);
    Point b({1, 0, 2});

    std::vector<Point> u;
    u.push_back(Point(0.0, 0.0, 1.0));
    u.push_back(Point(0.0, 1.0, 0.0));
    u.push_back(Point(1.0, 0.0, 0.0));
    u.push_back(Point(1.0, 1.0, 0.0));
    u.push_back(Point(2.0, 0.0, 0.0));
    u.push_back(Point(2.0, 1.0, 0.0));
    u.push_back(Point(3.0, 0.0, 0.0));
    u.push_back(Point(3.0, 1.0, 0.0));

    std::vector<Point> v;
    for (auto p : u)
        v.push_back(A * p + b);

    RotoTranslationMatrixHandler h(u, v);

    std::cout << " n_points  = " << h.get_number_of_points() << std::endl;
    std::cout << " volume    = " << are_equal(h.get_volume_diff() - 1.0) << std::endl;
    std::cout << " err_pt[0] = " << are_equal(h.get_error_on_point(0)) << std::endl;
    std::cout << " err_pt[3] = " << are_equal(h.get_error_on_point(3)) << std::endl;

    // Apply a mask that drops two of the eight points.
    std::vector<size_t> mask = {1, 0, 1, 1, 1, 0, 1, 1};
    h.update_mask(mask);

    std::cout << " masked_n  = " << h.get_number_of_points() << std::endl;
    std::cout << " masked_e  = " << are_equal(h.get_error()) << std::endl;
    std::cout << " masked_v  = " << are_equal(h.get_volume_diff() - 1.0) << std::endl;

#else  // MLIB_USE_EIGEN3
    make_test_pass("matrix/rototranslation_matrix_handler_01");
#endif // MLIB_USE_EIGEN3
}
