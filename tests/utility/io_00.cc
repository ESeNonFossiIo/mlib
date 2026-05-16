#include "../test.h"

#include "mlib/utility/io.h"

#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

using namespace mlib;

// Calls clean_screen() while routing stdout to /dev/null so the terminal
// escape sequence does not pollute the captured test output. We only care
// that the call executes (for coverage) and returns to the caller.
int main()
{
    print_title("clean_screen");

    std::cout << "Before clean_screen()" << std::endl;
    std::cout.flush();

    // Save fd 1, redirect to /dev/null, call, restore.
    fflush(stdout);
    const int saved_stdout = dup(1);
    const int devnull      = open("/dev/null", O_WRONLY);
    if (devnull >= 0)
    {
        dup2(devnull, 1);
        close(devnull);

        clean_screen();

        fflush(stdout);
        dup2(saved_stdout, 1);
    }
    close(saved_stdout);

    std::cout << "After clean_screen()" << std::endl;

    return 0;
}
