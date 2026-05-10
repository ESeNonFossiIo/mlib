#include "../test.h"

#include "mlib/core/version.h"

#include <iostream>
#include <string>

using namespace mlib;

int main()
{
    print_title("Version");

    std::string v(version());
    std::size_t maj = version_major();
    std::size_t min = version_minor();
    std::size_t pat = version_patch();

    // Avoid printing the version itself (it changes between releases).
    // Instead, only print structural facts that are stable across versions.
    std::cout << " has_version_string = " << (v.empty() ? 0 : 1) << std::endl;
    std::cout << " components_match   = "
              << ((std::to_string(maj) + "." + std::to_string(min) + "." + std::to_string(pat)) == v
                      ? 1
                      : 0)
              << std::endl;

    return 0;
}
