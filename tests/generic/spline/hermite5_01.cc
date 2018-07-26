#include "../../test_compare.h"
#include "../../test.h"

#include "mlib/math/point.h"
#include "mlib/math/spline.h"
#include <iostream>
#include <iomanip>

using namespace _mlib;

int main()
{
  print_title("Hermite Spline");


  unsigned int j_end = 100;

  Point p0(0,0,0);
  Point v0(1,0,0);
  Point a0(0,-1,0);
  double t0(0.0);

  Point p1(1,1,1);
  Point v1(0,1,0);
  Point a1(1,0,0);
  double t1(2.5);

  double f = t1 - t0;
  v0 *= f;
  v1 *= f;
  a0 *= f*f;
  a1 *= f*f;

  HermiteSpline5<3> hs(p0,p1,v0,v1,a0,a1,t0,t1);

  for(unsigned int j = 0; j <= j_end; j++)
    {
      double t  =  t0  + ((double)j/(double)j_end)  * (t1 - t0);
      std::cout << std::setprecision(8) << " " << std::setw(10) << t;
      std::cout << std::setprecision(8) << " " << std::setw(10) <<
                hs.get_normalized_time(t);

      std::cout << std::setprecision(8) << " " << std::setw(10) << hs.p(t).x();
      std::cout << std::setprecision(8) << " " << std::setw(10) << hs.p(t).y();
      std::cout << std::setprecision(8) << " " << std::setw(10) << hs.p(t).z();

      std::cout << std::setprecision(2) << " " << std::setw(10) << 1e5 * hs.v(t).x();
      std::cout << std::setprecision(2) << " " << std::setw(10) << 1e5 * hs.v(t).y();
      std::cout << std::setprecision(2) << " " << std::setw(10) << 1e5 * hs.v(t).z();
      std::cout << std::endl << std::flush;
    }

  return 0;
}
