#include "../test.h"

#include "mlib/utility/types.h"

#include <iostream>
#include <string>

using namespace mlib;

int main()
{
    print_title("Zeros (specializations)");

    std::string s = zero<std::string>();
    bool b = zero<bool>();

    std::cout << " string_is_empty = " << (s.empty() ? 1 : 0) << std::endl;
    std::cout << " bool_value      = " << (b ? 1 : 0) << std::endl;

    return 0;
}
