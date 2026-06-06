#include "../test.h"

#include "numerix/math/euclidean_geometry.h"

#include <iomanip>
#include <iostream>

using namespace numerix;

int main()
{
    print_title("EuclideanGeometry - normal_form + accessors");

    std::cout << std::fixed << std::setprecision(3);

    // HyperPlane with normal_form=true → triggers normal_form() method
    // {2, 4, 6} normalised to {1, 2, 3} (divide by first element)
    HyperPlane hp({2.0, 4.0, 6.0}, true);
    std::cout << " hp.a  = " << hp.a() << std::endl;
    std::cout << " hp.b  = " << hp.b() << std::endl;
    std::cout << " hp.c  = " << hp.c() << std::endl;

    // HyperPlane with 4 elements for hp.d()
    HyperPlane hp4({3.0, 6.0, 9.0, 12.0}, true);
    std::cout << " hp4.d = " << hp4.d() << std::endl;

    return 0;
}
