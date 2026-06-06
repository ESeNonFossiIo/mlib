#include "../test.h"

#include "numerix/math/convolution.h"

#include <iomanip>
#include <iostream>

using namespace numerix;

int main()
{
    print_title("Convolution - GaussKernel evaluation");

    std::cout << std::fixed << std::setprecision(5);

    // GaussKernel with explicit mu — covers lambda body (lines 70-71 of convolution.cc)
    GaussKernel gauss(1.0, 0.0);
    auto fn = gauss.get_kernel();
    std::cout << " gauss(0) = " << fn(0.0) << std::endl;
    std::cout << " gauss(1) = " << fn(1.0) << std::endl;

    // Use in a Convolution so the kernel function is called during compute
    Convolution conv(gauss);
    std::vector<double> in = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> out = conv(in);
    std::cout << " out_size = " << out.size() << std::endl;

    return 0;
}
