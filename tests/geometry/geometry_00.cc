#include "../test.h"

#include "mlib/math/geometry.h"

#include <iomanip>
#include <iostream>
#include <vector>

using namespace mlib;

namespace {

void dump(const std::string& label, const std::vector<std::size_t>& v)
{
    std::cout << label << " = [";
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        if (i)
            std::cout << ", ";
        std::cout << v[i];
    }
    std::cout << "]" << std::endl;
}

void dump(const std::string& label,
          const std::vector<std::pair<std::size_t, std::size_t>>& v)
{
    std::cout << label << " = [";
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        if (i)
            std::cout << ", ";
        std::cout << "(" << v[i].first << "," << v[i].second << ")";
    }
    std::cout << "]" << std::endl;
}

void dump(const std::string& label, const std::vector<double>& v)
{
    std::cout << label << " = [";
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        if (i)
            std::cout << ", ";
        std::cout << v[i];
    }
    std::cout << "]" << std::endl;
}

} // namespace

int main()
{
    print_title("geometry");

    std::cout << std::fixed << std::setprecision(3);

    // Build a piecewise-linear sample: 30 collinear points, then a kink.
    std::vector<double> x;
    std::vector<double> y;
    for (std::size_t i = 0; i < 30; ++i)
    {
        x.push_back(static_cast<double>(i));
        y.push_back(2.0 * static_cast<double>(i));
    }
    for (std::size_t i = 30; i < 50; ++i)
    {
        x.push_back(static_cast<double>(i));
        y.push_back(2.0 * 30.0 + 0.5 * (static_cast<double>(i) - 30.0));
    }

    dump("rectify           ", rectify(x, y, 0.001, 5));
    dump("uniform_grid(10)  ", extract_uniform_grid(x, 10));
    dump("linear_parts      ", extract_linear_parts(x, y, 0.0005, 5));

    // Two long collinear runs separated by one outlier point — exercises the
    // coalesce branch in extract_linear_parts (matching slope, gap < 2). A
    // tail of differently-sloped points keeps the inner while bounded so the
    // (i+j < y.size()) guard does not reject the run.
    std::vector<double> lx;
    std::vector<double> ly;
    for (std::size_t i = 0; i < 20; ++i)
    {
        lx.push_back(static_cast<double>(i));
        ly.push_back(3.0 * static_cast<double>(i));
    }
    lx.push_back(20.0);
    ly.push_back(3.0 * 20.0 + 5.0); // outlier
    for (std::size_t i = 21; i < 40; ++i)
    {
        lx.push_back(static_cast<double>(i));
        ly.push_back(3.0 * static_cast<double>(i));
    }
    for (std::size_t i = 40; i < 50; ++i)
    {
        lx.push_back(static_cast<double>(i));
        ly.push_back(3.0 * 40.0 - 5.0 * (static_cast<double>(i) - 40.0)); // kink
    }
    dump("linear_parts(2x) ", extract_linear_parts(lx, ly, 0.0005, 5));

    // Monotonic sequence for the merge_*_parts functions.
    std::vector<double> mx;
    std::vector<double> my_inc;
    std::vector<double> my_dec;
    for (std::size_t i = 0; i < 10; ++i)
    {
        mx.push_back(static_cast<double>(i));
        my_inc.push_back(static_cast<double>(i));     // strictly increasing
        my_dec.push_back(-static_cast<double>(i));    // strictly decreasing
    }

    dump("merge_inc (strict)", merge_increasing_parts(mx, my_inc, false));
    dump("merge_inc (>=)    ", merge_increasing_parts(mx, my_inc, true));
    dump("merge_dec (strict)", merge_decreasing_parts(mx, my_dec, false));
    dump("merge_dec (<=)    ", merge_decreasing_parts(mx, my_dec, true));

    // Interpolate over a couple of intervals — the second interval lies fully
    // inside the data so the inner loop runs.
    std::vector<std::pair<std::size_t, std::size_t>> idx;
    idx.push_back(std::make_pair(0, 5));
    idx.push_back(std::make_pair(5, 9));
    // Out-of-range pair: must be skipped by the `i_end < y.size()` guard.
    idx.push_back(std::make_pair(0, mx.size()));
    dump("interp_straight   ", interpolate_straight_parts(mx, my_inc, idx));

    return 0;
}
