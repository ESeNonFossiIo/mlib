#include "../test.h"

#include "numerix/utility/string.h"

#include <iostream>

using namespace numerix;

int main()
{
    print_title(" TEST for Utility - trim");

    std::string prova("abs \t n\n asdf \n  \n sdf  fasd ");

    std::cout << prova << std::endl;
    std::cout << trim(prova) << std::endl;

    return 0;
}
