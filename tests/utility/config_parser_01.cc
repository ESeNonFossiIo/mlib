#include "../test.h"

#include "numerix/utility/config_file.h"

#include <cstdio>  // std::remove
#include <fstream>
#include <iostream>

using namespace numerix;

// Exercises every typed ParsedParameters::add_new_entry specialisation
// (double, int, size_t, string, bool) for both:
//   * the already-present key path (val != "");
//   * the missing-key path (val == ""), which goes through add_entry;
// plus the range-out-of-bounds path that returns the default value, and
// the save-on-exit path.
int main()
{
    std::cout << "=================================================" << std::endl;
    std::cout << "  TEST for utility - ParsedParameters 01" << std::endl;
    std::cout << "=================================================" << std::endl;

    const std::string ini_in  = get_test_dir() + "/utility/config_parser_01.ini";
    const std::string ini_out = get_test_dir() + "/utility/config_parser_01_out.ini";

    // save_on_exit=true so we hit ParsedParameters::save and write to ini_out.
    ParsedParameters pp(ini_in, ini_out, /*save_on_exit=*/true);

    // Present keys, in-range values — last argument is min, second-to-last max.
    double d_ok    = pp.add_new_entry("section1", "double_in_range", 0.0, 10.0, -10.0);
    int    i_ok    = pp.add_new_entry("section1", "int_in_range",    0,   100, -100);
    std::size_t s_ok = pp.add_new_entry("section1", "size_in_range", (std::size_t)0,
                                        (std::size_t)100, (std::size_t)0);

    // Present keys, out-of-range values — must return the default.
    double d_bad = pp.add_new_entry("section1", "double_too_high", 7.0, 10.0, -10.0);
    int    i_bad = pp.add_new_entry("section1", "int_too_low",     1,   100,  0);
    std::size_t s_bad = pp.add_new_entry("section1", "size_too_high", (std::size_t)2,
                                         (std::size_t)100, (std::size_t)0);

    // Missing-key path for the numeric specialisations — exercises the
    // val == "" -> add_entry branch.
    int    i_missing  = pp.add_new_entry("section1", "int_missing",       42,
                                         100, -100);
    std::size_t s_missing = pp.add_new_entry("section1", "size_missing",
                                             (std::size_t)11,
                                             (std::size_t)100,
                                             (std::size_t)0);
    double d_missing  = pp.add_new_entry("section1", "double_missing",    1.25,
                                         10.0, -10.0);
    std::cout << " i_missing   = " << i_missing  << std::endl;
    std::cout << " s_missing   = " << s_missing  << std::endl;
    std::cout << " d_missing   = " << d_missing  << std::endl;

    // String and bool specialisations (present + missing key both).
    std::string  str_present = pp.add_new_entry<std::string>(
        "section1", "string_val", std::string("default"), std::string(""), std::string(""));
    std::string  str_missing = pp.add_new_entry<std::string>(
        "section1", "string_missing", std::string("fallback"),
        std::string(""), std::string(""));
    bool         b_present   = pp.add_new_entry("section1", "bool_val", false, true, false);
    bool         b_missing   = pp.add_new_entry("section1", "bool_missing", false, true, false);

    std::cout << " d_ok        = " << d_ok        << std::endl;
    std::cout << " i_ok        = " << i_ok        << std::endl;
    std::cout << " s_ok        = " << s_ok        << std::endl;
    std::cout << " d_bad->def  = " << d_bad       << std::endl;
    std::cout << " i_bad->def  = " << i_bad       << std::endl;
    std::cout << " s_bad->def  = " << s_bad       << std::endl;
    std::cout << " str_present = " << str_present << std::endl;
    std::cout << " str_missing = " << str_missing << std::endl;
    std::cout << " b_present   = " << b_present   << std::endl;
    std::cout << " b_missing   = " << b_missing   << std::endl;

    // Trigger save() -> writes ini_out.
    pp.save();

    // Tidy up the on-disk artefact so re-runs are idempotent.
    std::remove(ini_out.c_str());

    return 0;
}
