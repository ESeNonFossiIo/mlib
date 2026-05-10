#include "../test.h"

#include "mlib/math/convolution.h"

#include <iostream>

using namespace mlib;

int main()
{
    print_title("Convolution (extended)");

    // ---- Kernel construction and accessors ----
    Kernel k([](double x) { return x + 1.0; }, 0, 0);

    std::pair<int, int> sup = k.get_int_support();
    std::cout << " support        = (" << sup.first << "," << sup.second << ")" << std::endl;
    std::cout << " kernel_at_0    = " << k(0.0) << std::endl;
    std::cout << " kernel_at_3    = " << k(3.0) << std::endl;

    auto fn = k.get_kernel();
    std::cout << " get_kernel_at5 = " << fn(5.0) << std::endl;

    // ---- Convolution::compute via operator() ----
    // With support (0,0), the algorithm multiplies each in[i] by ker[0]=k(0)=1
    // for i = 0..in.size()-2 and leaves the last entry as the input.
    Convolution conv(k);
    std::vector<double> in_vec = {2.0, 4.0, 6.0, 8.0};
    std::vector<double> out_vec = conv(in_vec);

    std::cout << " conv_size      = " << out_vec.size() << std::endl;
    for (std::size_t i = 0; i < out_vec.size(); ++i)
        std::cout << "  out[" << i << "] = " << out_vec[i] << std::endl;

    // Same path through compute() directly
    std::vector<double> out_direct = conv.compute(in_vec);
    std::cout << " direct_eq      = " << (out_direct == out_vec ? 1 : 0) << std::endl;

    // ---- remove_small_values ----
    std::vector<double> noisy = {0.001, 0.5, 0.0005, 1.5, -0.002};
    std::vector<double> cleaned = remove_small_values(noisy, 0.01);
    std::cout << " cleaned_size   = " << cleaned.size() << std::endl;
    for (std::size_t i = 0; i < cleaned.size(); ++i)
        std::cout << "  cleaned[" << i << "] = " << cleaned[i] << std::endl;

    // ---- convolve free function (smoke test, doesn't crash) ----
    std::vector<double> in = {1, 2, 3, 4, 5};
    std::vector<double> cv = convolve(in, 1);
    std::cout << " convolve_size  = " << cv.size() << std::endl;

    return 0;
}
