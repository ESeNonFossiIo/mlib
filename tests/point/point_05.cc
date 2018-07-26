#include "../test.h"

#include "mlib/math/geometry/clothoid.h"
#include "mlib/math/matrix/rotation.h"

using namespace mlib;

int main()
{
  print_title("Point rotation");


  double d            = 4.0;
  double t_p          = M_PI / 3.0;
  double t_q          = M_PI / 1.0;

  Point p1(1.0,1.0);
  Point p2(d * std::cos(t_p), d * std::sin(t_p));
  p2 += p1;

  Point q1(0.0,0.0);
  Point q2(d * std::cos(t_q), d * std::sin(t_q));
  q2 += q1;


  std::cout << p1;
  std::cout << q1;
  std::cout << "-------------------------------------------------------------"
            << std::endl;
  std::cout << p2;
  std::cout << q2;
  std::cout << "-------------------------------------------------------------"
            << std::endl;
  std::cout << "-------------------------------------------------------------"
            << std::endl;

  double s = ((p2 - p1).t() * (q2 - q1))[0];
  s /= (p2 - p1).l_2_norm();
  s /= (q2 - q1).l_2_norm();
  double alpha = std::acos(s);


  Point t = p1 - q1;
  Angle beta(-1*alpha);
  Rotation2DMatrix R(beta);

  std::cout << R*(q1 - q1) + q1 + t;
  std::cout << p1;

  std::cout << R*(q2 - q1) + q1 + t;
  std::cout << p2;

}
