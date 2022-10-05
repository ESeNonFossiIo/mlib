#include "mlib/math/point.h"
#include "mlib/math/angle.h"

#include "mlib/math/matrix/rotation.h"
#include "mlib/math/spline.h"

#include <iostream>
#include <iomanip>

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
  Point p(411753.7782, 4544475.235);
  // initial yaw
  Angle py(35.870, AngleType::deg);
  // initial time
  double pt(25900.177);

  // final position
  Point q(411753.7857, 4544475.244);
  // final yaw
  Angle qy(35.852, AngleType::deg);
  // final time
  double qt(25900.182);

  Point ax(0.0,1.0);
  bool clock_wise = true;

  double reparametrize = (p-q).l_2_norm();
  std::cout << " --> " <<  reparametrize << std::endl;


////////////////////////////////////////////////////////////////////////////////

  Point tp = convert_angle_to_vector(py, ax, clock_wise);
  Point tq = convert_angle_to_vector(qy, ax, clock_wise);

  tp *= reparametrize;
  tq *= reparametrize;

  HermiteSpline<2> hs(p, q, tp, tq, pt, qt);
  LinearInterpolation<2> li(p, q, pt, qt);
  LinearInterpolation<2> tli(tp, tq, pt, qt);

  double ratio = 1.0;
  unsigned int i_end = 1000;

  for(unsigned int i = 0; i < i_end; i++)
    {
      double t  =  pt  + ((double)i/(double)i_end)  * (qt - pt)/ratio;

      std::cout << std::setw(15) << t << std::flush;
      std::cout << std::setw(15) << hs.p(t).x() - 411753;
      std::cout << std::setw(15) << hs.p(t).y() - 4544475;
      std::cout << std::setw(15) << hs.v(t).x();
      std::cout << std::setw(15) << hs.v(t).y();
      std::cout << std::setw(15) << convert_vector_to_angle(hs.v(t), ax,
                                                            clock_wise).deg();
      std::cout << std::setw(15) << convert_vector_to_angle(hs.p(qt)-hs.p(pt), ax,
                                                            clock_wise).deg();
      std::cout << std::setw(15) << li(t).x()- 463915;
      std::cout << std::setw(15) << li(t).y() - 5264223;
      std::cout << std::setw(15) << tli(t).x();
      std::cout << std::setw(15) << tli(t).y();
      std::cout << std::setw(15) << convert_vector_to_angle(tli(t), ax,
                                                            clock_wise).deg()
                << std::endl;
    }

  return 0;
}
