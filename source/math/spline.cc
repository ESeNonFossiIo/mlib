#include <mlib/core/unused.h>
#include <mlib/math/spline.h>

namespace mlib
{

  template<int dim>
  BaseInterpolation<dim>::
  BaseInterpolation(const double& t1_, const double& t2_)
    :
    t1(t1_),
    t2(t2_),
    normalize_time(normalize_range(t1,t2))
  {}

  template<int dim>
  void
  BaseInterpolation<dim>::
  is_valid_time(const double& t_) const
  {
    MLIB_UNUSED(t_);
    assert(t_ >= t1);
    assert(t_ <= t2);
  }

  template<int dim>
  double
  BaseInterpolation<dim>::
  get_normalized_time(const double& t_) const
  {
    assert(t_ >= t1);
    assert(t_ <= t2);
    return normalize_time(t_);
  }


  template class BaseInterpolation<1>;
  template class BaseInterpolation<2>;
  template class BaseInterpolation<3>;

////////////////////////////////////////////////////////////////////////////////

  template<int dim>
  LinearInterpolation<dim>::
  LinearInterpolation(const Point& p1_, const Point& p2_,
                      const double& t1_, const double& t2_)
    :
    BaseInterpolation<dim>(t1_,t2_),
    p1(p1_),
    p2(p2_)
  {}

  template<int dim>
  Point
  LinearInterpolation<dim>::
  operator()(const double& t_) const
  {
    this->is_valid_time(t_);
    double t = this->normalize_time(t_);
    return (1-t)*p1 + t * p2;
  }

  template class LinearInterpolation<1>;
  template class LinearInterpolation<2>;
  template class LinearInterpolation<3>;

////////////////////////////////////////////////////////////////////////////////

  template<int dim>
  HermiteSpline<dim>::
  HermiteSpline(const Point& p1_, const Point& p2_,
                const Point& v1_, const Point& v2_,
                const double& t1_, const double& t2_)
    :
    BaseInterpolation<dim>(t1_,t2_),
    p1(p1_),
    p2(p2_),
    v1(v1_),
    v2(v2_),
    h03(
  {
    1,0,-3,2
  }),
  h13({0,1,-2,1})
  {}

  template<int dim>
  Point
  HermiteSpline<dim>::
  p(const double& t_) const
  {
    this->is_valid_time(t_);
    double t = this->normalize_time(t_);

    Point p;
    p.resize(dim);
    p += h03(t)   * p1;
    p += h13(t)   * v1;
    p -= h13(1-t) * v2;
    p += h03(1-t) * p2;

    return p;
  }

  template<int dim>
  Point
  HermiteSpline<dim>::
  v(const double& t_) const
  {
    this->is_valid_time(t_);
    double t = this->normalize_time(t_);

    Point v;
    v.resize(dim);
    v += h03.d(1)(t)    * p1;
    v += h13.d(1)(t)    * v1;
    v += h13.d(1)(1-t)  * v2;
    v -= h03.d(1)(1-t)  * p2;

    return v;
  }

  template<int dim>
  Point
  HermiteSpline<dim>::
  a(const double& t_) const
  {
    this->is_valid_time(t_);
    double t = this->normalize_time(t_);

    Point a;
    a.resize(dim);
    a += h03.d(2)(t)    * p1;
    a += h13.d(2)(t)    * v1;
    a -= h13.d(2)(1-t)  * v2;
    a += h03.d(2)(1-t)  * p2;

    return a;
  }

  template class HermiteSpline<1>;
  template class HermiteSpline<2>;
  template class HermiteSpline<3>;

////////////////////////////////////////////////////////////////////////////////

  template<int dim>
  HermiteSpline5<dim>::
  HermiteSpline5(const Point& p1_, const Point& p2_,
                 const Point& v1_, const Point& v2_,
                 const Point& a1_, const Point& a2_,
                 const double& t1_, const double& t2_)
    :
    BaseInterpolation<dim>(t1_,t2_),
    p1(p1_),
    p2(p2_),
    v1(v1_),
    v2(v2_),
    a1(a1_),
    a2(a2_),
    h05(
  {
    1,  0,    0,    -10,   15,  -6
  }),
  h15({0,  1,    0,    -6,     8,  -3}),
      h25({0,  0,   .5,  -1.5,   1.5, -.5})
  {}

  template<int dim>
  Point
  HermiteSpline5<dim>::
  p(const double& t_) const
  {
    this->is_valid_time(t_);
    double t = this->normalize_time(t_);

    Point p;
    p.resize(dim);
    p += h05(t)   * p1;
    p += h15(t)   * v1;
    p += h25(t)   * a1;
    p += h25(1-t) * a2;
    p -= h15(1-t) * v2;
    p += h05(1-t) * p2;

    return p;
  }

  template<int dim>
  Point
  HermiteSpline5<dim>::
  v(const double& t_) const
  {
    this->is_valid_time(t_);
    double t = this->normalize_time(t_);

    Point v;
    v.resize(dim);
    v += h05.d(1)(t)    * p1;
    v += h15.d(1)(t)    * v1;
    v += h25.d(1)(t)    * a1;
    v -= h25.d(1)(1-t)  * a2;
    v += h15.d(1)(1-t)  * v2;
    v -= h05.d(1)(1-t)  * p2;

    return v;
  }

  template<int dim>
  Point
  HermiteSpline5<dim>::
  a(const double& t_) const
  {
    this->is_valid_time(t_);
    double t = this->normalize_time(t_);

    Point a;
    a.resize(dim);
    a += h05.d(2)(t)    * p1;
    a += h15.d(2)(t)    * v1;
    a += h25.d(2)(t)    * a1;
    a += h25.d(2)(1-t)  * a2;
    a -= h15.d(2)(1-t)  * v2;
    a += h05.d(2)(1-t)  * p2;

    return a;
  }

  template class HermiteSpline5<1>;
  template class HermiteSpline5<2>;
  template class HermiteSpline5<3>;
}
