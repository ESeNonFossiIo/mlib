#include "../test.h"

#include "numerix/math/utility.h"

#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace numerix;

namespace {

template <typename T>
void dump(const std::string& label, const std::vector<T>& v)
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

// Fills the gaps in math/utility.cc coverage:
//   * truncate_vec (template)
//   * normalize_range
//   * flat_part
//   * interpolate
//   * apply_lambda
//   * remove_singularities with left=false (right-fill branch)
int main()
{
    print_title("Math Utility - 11");

    std::cout << std::fixed << std::setprecision(3);

    // --- truncate_vec --------------------------------------------------
    std::vector<int> ivec = {-5, 0, 3, 7, 15};
    auto ivec_t = truncate_vec<int>(ivec, 0, 10);
    dump("trunc_vec(i)", ivec_t);

    std::vector<double> dvec = {-1.5, 0.0, 0.5, 2.5};
    auto dvec_t = truncate_vec<double>(dvec, 0.0, 2.0);
    dump("trunc_vec(d)", dvec_t);

    // --- normalize_range -----------------------------------------------
    auto nrm = normalize_range<double>(2.0, 4.0);
    std::cout << "norm(2.0)   = " << nrm(2.0) << std::endl;
    std::cout << "norm(3.0)   = " << nrm(3.0) << std::endl;
    std::cout << "norm(4.0)   = " << nrm(4.0) << std::endl;

    // --- flat_part -----------------------------------------------------
    // Long flat run, then a kink — exercises both the push branch and the
    // tail "if (v.back() != size-1)" branch.
    std::vector<double> flat = {1, 1, 1, 1, 1, 1, 1, 1, 5, 6, 7, 8,
                                9, 10, 11, 12, 13, 14, 15, 16};
    dump("flat_part   ", flat_part(flat, 0.001, 2));

    // --- interpolate ---------------------------------------------------
    // interpolate reads vec[j_init + j_end], so the working vector must be
    // at least max(idx[i] + idx[i+1])+1 long.  Use a 20-element ramp.
    std::vector<double> v_in;
    for (int k = 0; k < 20; ++k)
        v_in.push_back(static_cast<double>(k));
    std::vector<int> idx = {0, 3, 6, 9};
    dump("interpolate ", interpolate(v_in, idx, 0.5));

    std::vector<double> v_jagged(20, 0.0);
    for (std::size_t k = 0; k < v_jagged.size(); ++k)
        v_jagged[k] = (k % 2 == 0) ? 0.0 : 5.0;
    dump("interp(jag) ", interpolate(v_jagged, idx, 0.5));

    // --- difference (left=false) — both overloads ---------------------
    std::vector<double> dv = {1, 2, 4, 7, 11};
    auto diff_step  = difference(dv, /*step=*/1.0, /*left=*/false);
    dump("diff(step,r)", diff_step);

    std::vector<double> dy = {1, 2, 4, 7, 11};
    std::vector<double> dx = {0, 1, 2, 3, 4};
    auto diff_xy    = difference(dy, dx, /*left=*/false);
    dump("diff(xy ,r) ", diff_xy);

    // --- accumulate (left=false) ---------------------------------------
    std::vector<double> av = {1, 2, 3, 4, 5};
    auto acc_r = accumulate(av, /*step=*/1.0, /*left=*/false);
    dump("accum(r)    ", acc_r);

    // --- apply_lambda --------------------------------------------------
    std::vector<double> al_in = {1, 2, 3, 4};
    std::function<double(double)> sq = [](double x) { return x * x; };
    dump("apply_lambda", apply_lambda(al_in, sq));

    // --- remove_singularities with left=false --------------------------
    // Pattern: aligned-before, single jump, aligned-after.
    std::vector<double> sing = {0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0};
    dump("remove_sing ",
         remove_singularities(sing,
                              /*toll_zero=*/0.1,
                              /*toll_jump=*/0.5,
                              /*item_before=*/3,
                              /*item_after=*/3,
                              /*singularity_lenght=*/1,
                              /*left=*/false));

    return 0;
}
