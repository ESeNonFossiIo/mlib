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
  Point p(463914.88036014238605, 5264222.1359729496762);
  // initial yaw
  Angle py(-138.69499999999999318, AngleType::deg);
  // initial time
  double pt(307393.02000000001863);

  // final position
  Point q(463914.81817255466012, 5264222.0607811696827);
  // final yaw
  Angle qy(-138.67500000000001137, AngleType::deg);
  // final time
  double qt(307393.03000000002794);


////////////////////////////////////////////////////////////////////////////////

  Point tp = convert_angle_to_vector(py, Point(0.0,1.0), true);
  Point tq = convert_angle_to_vector(qy, Point(0.0,1.0), true);

  tp *= (p-q).l_2_norm();
  tq *= (p-q).l_2_norm();

  HermiteSpline<2> hs(p, q, tp, tq, pt, qt);
  LinearInterpolation<2> li(p, q, pt, qt);
  LinearInterpolation<2> tli(tp, tq, pt, qt);

  double t = 307393.02412856800947;

  std::cout << std::setw(15) << t - 307392 << std::flush;
  std::cout << std::setw(15) << hs.p(t).x() - 463915;
  std::cout << std::setw(15) << hs.p(t).y() - 5264223;
  std::cout << std::setw(15) << hs.v(t).x();
  std::cout << std::setw(15) << hs.v(t).y();
  std::cout << std::setw(15) << convert_vector_to_angle(hs.v(t), Point(0.0,
                                                                       1.0), true).deg() << std::endl;
  std::cout << std::setw(15) << t - 307392 << std::flush;
  std::cout << std::setw(15) << li(t).x()- 463915;
  std::cout << std::setw(15) << li(t).y() - 5264223;
  std::cout << std::setw(15) << tli(t).x();
  std::cout << std::setw(15) << tli(t).y();
  std::cout << std::setw(15) << convert_vector_to_angle(tli(t), Point(0.0,
                                                                      1.0), true).deg() << std::endl;

  return 0;
}
