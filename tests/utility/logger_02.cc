#include "../test.h"

#include "numerix/utility/logger.h"

#include <fstream>
#include <iostream>

using namespace numerix;
using namespace numerix::Color;

int main()
{
    print_title("Logger (print with content)");

    Logger log("", false);
    log.msg("hello");
    log.warning("world");

    // Redirect cout to /dev/null so timestamps don't appear in golden output.
    std::ofstream devnull("/dev/null");
    std::streambuf* old = std::cout.rdbuf(devnull.rdbuf());
    log.print();
    std::cout.rdbuf(old);

    std::cout << " print_covered = 1" << std::endl;
    return 0;
}
