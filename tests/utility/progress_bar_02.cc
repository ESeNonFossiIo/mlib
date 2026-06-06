#include "../test.h"

#include "numerix/utility/progress_bar.h"

#include <iostream>

using namespace numerix;

int main()
{
    print_title("ProgressBar (end)");

    ProgressBar pb(10);
    pb.print_bar(50);
    pb.end();
    std::cout << " end_covered = 1" << std::endl;
    return 0;
}
