#include "../test.h"

#include "mlib/math/math_sequence.h"

using namespace mlib;

int main()
{
  print_title("Math Sequence");

  {
    std::vector<int> init_sequence({1,1});
    std::function<int(std::vector<int>, int)> next_element = [](std::vector<int> x,
                                                                int n)
    {
      return x[0] + x[1];
    };

    MathSeq<int> ms(init_sequence, next_element);

    for(auto p: ms)
      {
        std::cout << p << " ";
      }

    ms.compute_elements(11);

    std::cout << std::endl;
    for(auto p: ms)
      {
        std::cout << p << " ";
      }
  }

  std::cout << std::endl;

  {
    std::vector<int> init_sequence({1});
    std::function<int(std::vector<int>, int)> next_element = [](std::vector<int> x,
                                                                int n)
    {
      return 2 * x[0];
    };

    MathSeq<int> ms(init_sequence, next_element);

    for(auto p: ms)
      {
        std::cout << p << " ";
      }
  }

  std::cout << std::endl;

  {
    std::vector<double> init_sequence({});
    std::function<double(std::vector<double>, int)> next_element = [](
                                                                     std::vector<double> x, int n)
    {
      return n;
    };

    MathSeq<double> ms(init_sequence, next_element);

    for(auto p: ms)
      {
        std::cout << p << " ";
      }
  }

  std::cout << std::endl;

  {
    std::vector<int> init_sequence({0, 1});
    std::function<int(std::vector<int>, int)> next_element = [](
                                                               std::vector<int> x, int n)
    {
      return x[1] + n;
    };

    MathSeq<int> ms(init_sequence, next_element);

    for(auto p: ms)
      {
        std::cout << p << " ";
      }
  }
}
