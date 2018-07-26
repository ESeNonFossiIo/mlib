#include "mlib/math/geometry/clothoid.h"
#include "mlib/math/utility.h"
#include "mlib/math/constants.h"

#include <cassert>  // std::assert

namespace mlib
{
  Clothoid::
  Clothoid(const double& a_,
           const Integral& integral_)
    :
    a(a_),
    integral(integral_)
  {};

  double
  Clothoid::
  l(const double& x) const
  {
    return a*x;
  }

  double
  Clothoid::
  k(const double& x) const
  {
    return x/a;
  }

  Point
  Clothoid::
  operator()(const double& x) const
  {
    Point p;
    p.resize(2);

    const std::function<double(double)>& s = [&](double u)
    {
      return this->a*std::sin(u*u);
    };


    const std::function<double(double)>& c = [&](double u)
    {
      return this->a*std::cos(u*u);
    };

    p[0] = integral(c,x);
    p[1] = integral(s,x);
    return p;
  }

  ApproximatedClothoid::
  ApproximatedClothoid(const double& a_)
    :
    a(a_)
  {};

  double
  ApproximatedClothoid::
  l(const double& x) const
  {
    return a*x;
  }

  double
  ApproximatedClothoid::
  k(const double& x) const
  {
    return x/a;
  }

  Point
  ApproximatedClothoid::
  operator()(const double& x) const
  {
    Point p;
    p.resize(2);

    const std::function<double(double)>& R = [&](double t)
    {
      return (0.506 * t + 1.0)/(1.79*t*t + 2.054*t + std::sqrt(2));
    };


    const std::function<double(double)>& A = [&](double t)
    {
      return (1.0)/(0.803 * t * t * t + 1.886 * t * t + 2.524 * t + 2);
    };

    p[0] = 0.5 - R(std::abs(x)) * std::sin(.5 * M_PI *(A(std::abs(x)) - x*x));
    p[1] = 0.5 - R(std::abs(x)) * std::cos(.5 * M_PI *(A(std::abs(x)) - x*x));
    return sgn(x) * M_PI * a * p;
  }
}
