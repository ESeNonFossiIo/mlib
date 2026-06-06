#include "../test.h"

#include "numerix/utility/logger.h"

#include <iostream>

using namespace numerix;
using namespace numerix::Color;

int main()
{
    print_title("Logger");

    Logger log("", true);

    log.write("Prova", "testo", white);
    log.warning("testo");
    log.error("testo");
    log.status("testo");
    log.value("testo", "3.14");
    log.msg("testo");

    return 0;
}
