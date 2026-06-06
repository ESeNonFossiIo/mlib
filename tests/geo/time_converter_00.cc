#include "../test.h"
#include "../test_compare.h"

#include "numerix/geo/time_converter.h"

#include <iostream>
#include <vector>

using namespace numerix;

int main()
{

    print_title("TimeConverter");

    TimeConverter tc(32.0, 7, 12, 8, 3, 2018);

    std::cout << " UTC Week -> " << tc.getUTCWeek() << std::endl;
}
