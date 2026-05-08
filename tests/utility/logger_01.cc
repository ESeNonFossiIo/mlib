#include "../test.h"

#include "mlib/utility/file.h"
#include "mlib/utility/logger.h"

#include <iostream>

using namespace mlib;
using namespace mlib::Color;

int main()
{
  print_title("Logger (file + copy + print)");

  // Logger that does NOT echo to stdout, so timestamped lines do not pollute
  // the test output.
  Logger log("", false);
  log.warning("a");
  log.error("b");
  log.status("c");
  log.value("d", "1");
  log.msg("e");
  log.write("RAW", "f", white);

  // ---- save_on_file: pass an explicit filename via the argument ----
  const std::string out = "logger_01.log";
  log.save_on_file(out);
  std::cout << " file_exists    = " << (file_exists(out) ? 1 : 0) << std::endl;
  std::cout << " line_count     = " << get_number_of_lines(out) << std::endl;

  // Re-saving with empty filename uses the previously-set name and is a no-op
  // for content (just rewrites). Still exercises the early-return path.
  log.save_on_file();
  std::cout << " resave_exists  = " << (file_exists(out) ? 1 : 0) << std::endl;

  // ---- copy constructor: copy preserves the content vector ----
  Logger copy(log);

  // Save the copy under a different name and check it has the same line count.
  const std::string out_copy = "logger_01_copy.log";
  copy.save_on_file(out_copy);
  std::cout << " copy_exists    = " << (file_exists(out_copy) ? 1 : 0) << std::endl;
  std::cout << " copy_line_eq   = "
            << (get_number_of_lines(out_copy) == get_number_of_lines(out) ? 1 : 0)
            << std::endl;

  // ---- print(): just verify it does not crash by calling it on an empty
  // logger. An empty logger writes nothing to stdout. ----
  Logger empty("", false);
  empty.print();
  std::cout << " print_done     = 1" << std::endl;

  return 0;
}
