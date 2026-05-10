#include "../test.h"

#include "mlib/utility/color.h"

#include <iostream>

using namespace mlib;
using namespace mlib::Color;

int main()
{
    print_title("Color (extended)");

    std::cout << black.init() << " black " << black.end() << std::endl;
    std::cout << magenta.init() << " magenta " << magenta.end() << std::endl;
    std::cout << cyan.init() << " cyan " << cyan.end() << std::endl;
    std::cout << light_gray.init() << " light_gray " << light_gray.end() << std::endl;
    std::cout << white.init() << " white " << white.end() << std::endl;

    // Default-constructed: color=32 (green), style=1
    GeneralColor def;
    std::cout << def.init() << " default " << def.end() << std::endl;

    return 0;
}
