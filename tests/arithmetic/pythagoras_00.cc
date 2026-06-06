#include "../test.h"

#include "numerix/math/arithmetic.h"
#include "numerix/utility/output.h"

using namespace numerix;

int main()
{
    print_title("Arithmetic - Pythagoras");

    std::cout << " sqrt(9 + 16) = 5 = " << pythagoras(3, 4) << std::endl;
}
