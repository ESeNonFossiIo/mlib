#include "../test.h"

#include "numerix/math/arithmetic.h"
#include "numerix/utility/output.h"

using namespace numerix;

int main()
{
    print_title("Arithmetic - decimal_to_binary");

    for (std::size_t i = 0; i < 15; ++i) {
        std::cout << " ========================================= " << std::endl;
        std::cout << "     " << i << " = " << decimal_to_binary(i) << std::endl;
        std::cout << "     " << i << " = " << binary_to_decimal(decimal_to_binary(i)) << std::endl;
        std::cout << " ========================================= " << std::endl;
    }
}
