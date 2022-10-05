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
  Point p(463915.8049, 5264223.272);
  // initial yaw
  Angle py(-138.963, AngleType::deg);
  // initial time
  double pt(307392.87);

  // final position
  Point q(463915.7443, 5264223.195);
  // final yaw
  Angle qy(-138.949, AngleType::deg);
  // final time
  double qt(307392.88);


  Point axes(0.0,1.0);
////////////////////////////////////////////////////////////////////////////////

  Point tp = convert_angle_to_vector(py, axes, true);
  Point tq = convert_angle_to_vector(qy, axes, true);

  double v = (q-p).l_2_norm();
  std::cout << " v = " << v << std::endl;

  tp*=v;
  tq*=v;

  HermiteSpline<2> hs(p, q, tp, tq, pt, qt);

  std::cout << std::setprecision(9)
            << std::setw(15) << "t"
            << std::setw(15) << "tn"
            << std::setw(15) << "x"
            << std::setw(15) << "y"
            << std::setw(15) << "tx"
            << std::setw(15) << "ty"
            << std::setw(15) << "yaw" << std::endl;

  std::size_t i_end = 100;
  for(std::size_t i = 0; i <= i_end; i++)
    {
      double t  =  pt  + ((double)i/(double)i_end)  * (qt - pt);
      std::cout << std::setw(15) << t - 307392 << std::flush;
      std::cout << std::setw(15) << hs.get_normalized_time(t) << std::flush;
      std::cout << std::setw(15) << hs.p(t).x() - 463915;
      std::cout << std::setw(15) << hs.p(t).y() - 5264223;
      std::cout << std::setw(15) << hs.v(t).x();
      std::cout << std::setw(15) << hs.v(t).y();
      std::cout << std::setw(15) << convert_vector_to_angle(hs.v(t), axes,
                                                            true).deg() << std::endl;
    }

  return 0;
}
