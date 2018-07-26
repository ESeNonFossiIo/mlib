#include "../test.h"

#include "mlib/math/geometry/segment.h"

using namespace _mlib;

int main()
{
  print_title("Segment - 01");

  {
    Point a(0,0);
    Point b(0,1);
    Segment ab(a,b);

    Point c(1,1);
    Point d(1,0);
    Segment cd(c,d);

    std::cout << " are_aligned = "
              << are_aligned(ab, cd)
              << std::endl;
  }

  {
    Point a(0,0);
    Point b(0,1);
    Segment ab(a,b);

    Point c(0,1.5);
    Point d(0,2);
    Segment cd(c,d);

    std::cout << " are_aligned = "
              << are_aligned(ab, cd)
              << std::endl;
  }

  {
    Point a(0,0);
    Point b(0,1);
    Segment ab(a,b);

    Point c(0,1.5);
    Point d(1,100);
    Segment cd(c,d);

    std::cout << " are_aligned = "
              << are_aligned(ab, cd)
              << std::endl;
  }

  {
    Point a(0,0);
    Point b(0,1);
    Segment ab(a,b);

    Point c(0,1.5);
    Point d(1,100);
    Segment cd(c,d);

    std::cout << " are_aligned = "
              << are_aligned(ab, cd, 0.2)
              << std::endl;
  }

  std::cout << " ================================================ "
            << std::endl;

  {
    Point a(0,0);
    Point b(0,1);
    Segment ab(a,b);

    Point c(1,1);
    Point d(1,0);
    Segment cd(d,c);

    std::cout << " are_aligned = "
              << are_aligned(ab, cd)
              << std::endl;
  }

  {
    Point a(0,0);
    Point b(0,1);
    Segment ab(a,b);

    Point c(0,1.5);
    Point d(0,2);
    Segment cd(d,c);

    std::cout << " are_aligned = "
              << are_aligned(ab, cd)
              << std::endl;
  }

  {
    Point a(0,0);
    Point b(0,1);
    Segment ab(a,b);

    Point c(0,1.5);
    Point d(1,100);
    Segment cd(d,c);

    std::cout << " are_aligned = "
              << are_aligned(ab, cd)
              << std::endl;
  }

  {
    Point a(0,0);
    Point b(0,1);
    Segment ab(a,b);

    Point c(0,1.5);
    Point d(1,100);
    Segment cd(d,c);

    std::cout << " are_aligned = "
              << are_aligned(ab, cd, 0.2)
              << std::endl;
  }
}
