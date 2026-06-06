#include "../test.h"

#include "numerix/math/math_sequence.h"

#include <iostream>
#include <vector>

using namespace numerix;

int main()
{
    print_title("MathSeq - compute_elements early return");

    std::vector<int> init = {1, 2, 3, 4, 5};
    auto next = [](std::vector<int> v, int) { return v.back() + 1; };

    // n_elements=2 < init.size()=5 → compute_elements hits early return
    MathSeq<int> seq(init, next, 0, 2);
    std::cout << " size = " << (seq.end() - seq.begin()) << std::endl;
    std::cout << " sum  = " << seq.get_sum() << std::endl;

    return 0;
}
