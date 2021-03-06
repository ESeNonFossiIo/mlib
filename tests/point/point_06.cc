#include "../test.h"

#include "mlib/math/geometry/clothoid.h"
#include "mlib/math/matrix/utility.h"

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


  Point t;
  Matrixd R;
  rototranslation_matrix_2d(p1,p2,q1,q2,R,t);


  std::cout << R*(q1 - q1) + q1 + t;
  std::cout << p1;

  std::cout << R*(q2 - q1) + q1 + t;
  std::cout << p2;

}
