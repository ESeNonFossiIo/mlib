#include "mlib/math/euclidean_geometry.h"

namespace _mlib
{

  HyperPlane::
  HyperPlane()
    :
    Point()
  {};

  HyperPlane::
  HyperPlane(std::initializer_list<double> list,
             bool normal_form)
    :
    Point(list)
  {
    if(normal_form)
      this->normal_form();
  };

  void
  HyperPlane::
  normal_form()
  {
    assert(this->elements[0]*this->elements[0]>0);

    for(size_t i = 1; i < this->dim(); ++i)
      this->elements[i]/= this->elements[0];

    this->elements[0]/= this->elements[0];
  };

  /**
   * @brief a
   * @return
   */
  double
  HyperPlane::
  a() const
  {
    return this->elements[0];
  };

  /**
   * @brief b
   * @return
   */
  double
  HyperPlane::
  b() const
  {
    return this->elements[1];
  };

  /**
   * @brief c
   * @return
   */
  double
  HyperPlane::
  c() const
  {
    return this->elements[2];
  };

  /**
   * @brief d
   * @return
   */
  double
  HyperPlane::
  d() const
  {
    return this->elements[3];
  };

////////////////////////////////////////////////////////////////////////////////

  HyperPlane
  middle_hyperplane_between_points(const Point& p,
                                   const Point& q)
  {
    HyperPlane hp;
    hp.resize(p.dim()+1);
    for(unsigned int i = 0; i < p.dim(); ++i)
      hp[i] = p[i] - q[i];
    for(unsigned int i = 0; i < p.dim(); ++i)
      hp[p.dim()] -= hp[i]*(p[i] + q[i])/2;
    if(hp[0]*hp[0] > VAR_MLIB_ZERO_TOLERANCE)
      {
        for(unsigned int i = 1; i < p.dim()+1; ++i)
          hp[i]/=hp[0];
        hp[0]/=hp[0];
      }
    return hp;
  };


////////////////////////////////////////////////////////////////////////////////

  _mlib::
  Point
  get_hyperplanes_intersection(const _mlib::HyperPlane& p,
                               const _mlib::HyperPlane& q)
  {
    _mlib::Matrix<double> m(2, 2);
    _mlib::Point b;
    b.resize(2);

    for(unsigned int i = 0; i<2; ++i)
      m(0,i) = p[i];
    for(unsigned int i = 0; i<2; ++i)
      m(1,i) = q[i];

    b[0] = -p[2];
    b[1] = -q[2];

    assert(m.det()*m.det() > VAR_MLIB_ZERO_TOLERANCE);

    return m.inv() * b;
  };

  Point
  get_hyperplanes_intersection(const HyperPlane& p,
                               const HyperPlane& q,
                               const HyperPlane& r)
  {
    Matrix<double> m(3, 3);
    Point b;
    b.resize(3);

    for(unsigned int i = 0; i<3; ++i)
      m(0,i) = p[i];
    for(unsigned int i = 0; i<3; ++i)
      m(1,i) = q[i];
    for(unsigned int i = 0; i<3; ++i)
      m(2,i) = r[i];

    b[0] = -p[3];
    b[1] = -q[3];
    b[2] = -r[3];

    assert(m.det()*m.det() > VAR_MLIB_ZERO_TOLERANCE);

    return m.inv() * b;
  };

  HyperPlane
  hyperplane_passing_through_three_points(const Point& a,
                                          const Point& b,
                                          const Point& c)
  {
    Point p1(b);
    p1 -= a;
    Point p2(c);
    p2 -= a;

    Matrix<double> m(3,3);

    for(unsigned int i = 0; i<3; ++i)
      m(0,i) = a[i];
    for(unsigned int i = 0; i<3; ++i)
      m(1,i) = p1[i];
    for(unsigned int i = 0; i<3; ++i)
      m(2,i) = p2[i];

    HyperPlane l;
    l.resize(a.dim()+1);
    l[0] = m.cofactor(0,0).det();
    l[1] = -m.cofactor(0,1).det();
    l[2] = m.cofactor(0,2).det();
    l[3] = - a[0]*l[0] + a[1]*l[1] - a[2]*l[2];

    return l;
  };

  Point
  circumference_center(const Point& a,
                       const Point& b,
                       const Point& c)
  {
    if(a.dim() == 3)
      {
        HyperPlane l1 = middle_hyperplane_between_points(a,b);
        HyperPlane l2 = middle_hyperplane_between_points(b,c);
        HyperPlane l3 = hyperplane_passing_through_three_points(a,b,c);
        return get_hyperplanes_intersection(l1,l2,l3);
      }
    else
      {
        HyperPlane l1 = _mlib::middle_hyperplane_between_points(a,b);
        HyperPlane l2 = _mlib::middle_hyperplane_between_points(b,c);
        return _mlib::get_hyperplanes_intersection(l1,l2);
      }
  };
}
