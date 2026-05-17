#include "../../test.h"

#include "mlib/utility/parser/ini.h"

#include <iostream>

using namespace mlib;

int main()
{
    print_title("INIHandler - typed get_val + comment line");

    // INI content with typed values and a comment line (starts with ';')
    INIHandler h("[sec]\n"
                 "; this is a comment line\n"
                 "dval  = 3.14\n"
                 "bval  = true\n"
                 "sval  = 42\n"
                 "ival  = -7\n",
                 false);

    // get_val<double> — if branch (section exists)
    std::cout << " dval = " << h.get_val<double>("sec", "dval", 0.0) << std::endl;

    // get_val<double> — else branch (section missing)
    std::cout << " dmiss = " << h.get_val<double>("missing", "x", 1.5) << std::endl;

    // get_val<bool> — if branch
    std::cout << " bval = " << h.get_val<bool>("sec", "bval", false) << std::endl;

    // get_val<bool> — else branch
    std::cout << " bmiss = " << h.get_val<bool>("missing", "x", false) << std::endl;

    // get_val<std::size_t> — if branch
    std::cout << " sval = " << h.get_val<std::size_t>("sec", "sval", 0UL) << std::endl;

    // get_val<std::size_t> — else branch
    std::cout << " smiss = " << h.get_val<std::size_t>("missing", "x", 99UL) << std::endl;

    // get_val<int> — if branch
    std::cout << " ival = " << h.get_val<int>("sec", "ival", 0) << std::endl;

    // get_val<int> — else branch
    std::cout << " imiss = " << h.get_val<int>("missing", "x", -1) << std::endl;

    return 0;
}
