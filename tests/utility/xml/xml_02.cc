#include "../../test.h"

#include "numerix/utility/parser/xml.h"

#include <iostream>

using namespace numerix;

int main()
{
    print_title("XMLHandler - default constructor");

    // XMLHandler() — default constructor — covers lines 65 and 67 of xml.cc
    XMLHandler h;
    std::cout << " created = 1" << std::endl;

    return 0;
}
