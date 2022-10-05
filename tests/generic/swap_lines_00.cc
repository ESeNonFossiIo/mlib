#include "../test.h"

#include "mlib/math/matrix/utility.h"

#include <iostream>

using namespace mlib;

int main()
{
  print_title("Integral");

  std::size_t size = 5;

  Matrixd m(size,size);
  Point p;
  p.resize(size);

  for(size_t i = 0; i < size; ++i)
    {
      for(size_t j = 0; j < size; ++j)
        {
          m(i,j) = i+j;
        }
    }

  for(size_t j = 0; j < size; ++j)
    {
      p[j] = j;
    }

  std::cout << m;
  std::cout << p;

  swap_lines(m,2,3);
  std::cout << m;

  swap_lines(m,p,2,3);
  std::cout << m;
  std::cout << p;

}
