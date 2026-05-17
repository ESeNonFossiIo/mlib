#include "../test.h"

#include "mlib/utility/io.h"

#include <cstdio>
#include <fcntl.h>
#include <iostream>
#ifdef _WIN32
#include <io.h>
static const char* k_null_dev = "NUL";
#define fd_dup  _dup
#define fd_dup2 _dup2
#define fd_open _open
#define fd_close _close
#else
#include <unistd.h>
static const char* k_null_dev = "/dev/null";
#define fd_dup  dup
#define fd_dup2 dup2
#define fd_open open
#define fd_close close
#endif

using namespace mlib;

// Calls clean_screen() while routing stdout to the null device so the
// terminal escape sequence does not pollute the captured test output.
int main()
{
    print_title("clean_screen");

    std::cout << "Before clean_screen()" << std::endl;
    std::cout.flush();

    // Save fd 1, redirect to null device, call, restore.
    fflush(stdout);
    const int saved_stdout = fd_dup(1);
    const int devnull      = fd_open(k_null_dev, O_WRONLY);
    if (devnull >= 0)
    {
        fd_dup2(devnull, 1);
        fd_close(devnull);

        clean_screen();

        fflush(stdout);
        fd_dup2(saved_stdout, 1);
    }
    fd_close(saved_stdout);

    std::cout << "After clean_screen()" << std::endl;

    return 0;
}
