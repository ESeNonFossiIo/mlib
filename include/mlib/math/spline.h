#ifndef __m_SPLINE_H__
#define __m_SPLINE_H__

#include "mlib/math/point.h"
#include "mlib/math/algebra.h"
#include "mlib/math/utility.h"

/** \addtogroup math
 *  @{
 */
namespace mlib
{

  template<int dim = 3>
  class BaseInterpolation
  {
  public:
    BaseInterpolation(const double& t1_, const double& t2_);

    void
    is_valid_time(const double& t_) const;

    double
    get_normalized_time(const double& t_) const;

    double t1, t2;
    std::function<double(double)> normalize_time;
  };

  template<int dim = 3>
  class LinearInterpolation : public BaseInterpolation<dim>
  {
  public:
    using BaseInterpolation<dim>::get_normalized_time;

    LinearInterpolation(const Point& p1_, const Point& p2_,
                        const double& t1_, const double& t2_);

    Point
    operator()(const double& t_) const;

    Point p1,p2,v1,v2;
  };


  template<int dim = 3>
  class HermiteSpline : public BaseInterpolation<dim>
  {
  public:
    using BaseInterpolation<dim>::get_normalized_time;

    HermiteSpline(const Point& p1_, const Point& p2_,
                  const Point& v1_, const Point& v2_,
                  const double& t1_, const double& t2_);

    Point p(const double& t_)const;
    Point v(const double& t_)const;
    Point a(const double& t_)const;

    Point p1,p2,v1,v2;

    Polynomial h03;
    Polynomial h13;
  };


  template<int dim = 3>
  class HermiteSpline5 : public BaseInterpolation<dim>
  {
  public:
    using BaseInterpolation<dim>::get_normalized_time;

    HermiteSpline5(const Point& p1_, const Point& p2_,
                   const Point& v1_, const Point& v2_,
                   const Point& a1_, const Point& a2_,
                   const double& t1_, const double& t2_);

    Point
    p(const double& t_) const;

    Point
    v(const double& t_) const;

    Point
    a(const double& t_) const;

    Point p1,p2,v1,v2,a1,a2;

    Polynomial h05;
    Polynomial h15;
    Polynomial h25;
  };

}
/** @}*/
#endif // __m_SPLINE_H__
