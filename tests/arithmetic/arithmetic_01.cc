#include "../test.h"

#include "numerix/math/arithmetic.h"

#include <iostream>

using namespace numerix;

int main()
{
    print_title("Arithmetic - missing branches");

    // binary_to_decimal with empty vector → return 0 early
    std::vector<std::size_t> empty;
    std::cout << " binary_to_decimal({}) = " << binary_to_decimal(empty) << std::endl;

    // pythagoras with |a| > |b| → takes the if-branch
    std::cout << " pythagoras(4,3) = " << pythagoras(4.0, 3.0) << std::endl;

    // pythagoras with b == 0 → ternary else, zero branch
    std::cout << " pythagoras(0,0) = " << pythagoras(0.0, 0.0) << std::endl;

    return 0;
}
