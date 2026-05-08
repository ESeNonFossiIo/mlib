#include "../test.h"

#include "mlib/math/math_sequence.h"

using namespace mlib;

int main()
{
  print_title("Math Sequence Sum");

  // Sum of the first 10 Fibonacci numbers (init {1,1}, length 10)
  // 1+1+2+3+5+8+13+21+34+55 = 143
  {
    std::vector<int> init_sequence({1, 1});
    std::function<int(std::vector<int>, int)> next_element = [](std::vector<int> x,
                                                                int /* n */)
    {
      return x[0] + x[1];
    };

    MathSeq<int> ms(init_sequence, next_element);
    std::cout << " sum_fib_10 = " << ms.get_sum() << std::endl;

    // After extending to 11, get_sum should be 232 (143 + 89)
    ms.compute_elements(11);
    std::cout << " sum_fib_11 = " << ms.get_sum() << std::endl;
  }

  // Sum of doubles 0..9 = 45
  {
    std::vector<double> init_sequence({});
    std::function<double(std::vector<double>, int)> next_element = [](
                                                                     std::vector<double> /* x */, int n)
    {
      return n;
    };

    MathSeq<double> ms(init_sequence, next_element);
    std::cout << " sum_0_9    = " << ms.get_sum() << std::endl;
  }

  return 0;
}
