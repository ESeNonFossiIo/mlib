#include "mlib/math/point.h"
#include "mlib/math/angle.h"

#include "mlib/math/matrix/rotation.h"
#include "mlib/math/spline.h"

#include <iostream>

using namespace mlib;

int main()
{
  std::cout <<
            "=================================================" <<
            std::endl;
  std::cout << "  TEST for Hermite Spline" << std::endl;
  std::cout <<
            "=================================================" <<
            std::endl;

  // initial position
  Point p(0.0, 0.866025404);
  // initial yaw
  Angle py(150, AngleType::deg);
  // initial time
  double pt(1.0);

  // final position
  Point q(.5, 0.0);
  // final yaw
  Angle qy(150, AngleType::deg);
  // final time
  double qt(3.0);

  Point ax(0.0,1.0);
////////////////////////////////////////////////////////////////////////////////

  Point tp = convert_angle_to_vector(py, ax);
  Point tq = convert_angle_to_vector(qy, ax);

  tp *= (p-q).l_2_norm();
  tq *= (p-q).l_2_norm();

  HermiteSpline<2> hs(p, q, tp, tq, pt, qt);


  std::size_t i_end = 20;
  for(std::size_t i = 0; i <= i_end; i++)
    {
      double t  =  pt  + ((double)i/(double)i_end)  * (qt - pt);
      std::cout << " t =   " << t << std::flush << std::endl;
      std::cout << " x =   " << hs.p(t).x() << std::endl;
      std::cout << " y =   " << hs.p(t).y() << std::endl;
      std::cout << " yaw = " << convert_vector_to_angle(hs.v(t),
                                                        ax).deg() << std::endl;
      std::cout << "-------------------------------------------------" << std::endl;
    }

  return 0;
}
