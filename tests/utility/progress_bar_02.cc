#include "../test.h"

#include "mlib/utility/progress_bar.h"

#include <iostream>

using namespace mlib;

int main()
{
    print_title("ProgressBar (end)");

    ProgressBar pb(10);
    pb.print_bar(50);
    pb.end();
    std::cout << " end_covered = 1" << std::endl;
    return 0;
}
