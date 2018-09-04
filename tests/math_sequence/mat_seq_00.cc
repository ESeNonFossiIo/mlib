#include "../test.h"

#include "mlib/math/math_sequence.h"

using namespace mlib;

int main()
{
  print_title("Math Sequence");

  {
    std::vector<int> init_sequence({1,1});
    std::function<int(std::vector<int>)> next_element = [](std::vector<int> x)
    {
      return x[0] + x[1];
    };

    MathSeq<int> ms(init_sequence, next_element);

    for(auto p: ms)
      {
        std::cout << p << " ";
      }
  }

  std::cout << std::endl;

  {
    std::vector<int> init_sequence({1});
    std::function<int(std::vector<int>)> next_element = [](std::vector<int> x)
    {
      return 2 * x[0];
    };

    MathSeq<int> ms(init_sequence, next_element);

    for(auto p: ms)
      {
        std::cout << p << " ";
      }
  }
}
