#include "mlib/math/geometry/segment.h"

#include "mlib/math/constants.h"

#include <cmath>
#include <cassert>  // std::assert

namespace mlib
{

  Segment::
  Segment()
    :
    p1(),
    p2(),
    length(-1.0)
  {}

  Segment::
  Segment(const Point& p1_, const Point& p2_)
  {
    update(p1_,p2_);
  }

  void
  Segment::
  update(const Point& p1_, const Point& p2_)
  {
    p1 = p1_;
    p2 = p2_;
    length = (p1_ - p2_).l_2_norm();
    if(length > 0)
      {
        d = p2 - p1;
        d /= length;
      }
  }

  Point
  Segment::
  get_direction() const
  {
    return d;
  }

  double
  Segment::
  get_length() const
  {
    return length;
  }

  std::pair<Point, Point>
  Segment::
  get_extreme_points() const
  {
    return std::make_pair<>(p1, p2);
  }

  Segment&
  Segment::
  operator*= (const double& a)
  {
    p1 *= a;
    p2 *= a;
    length *= a;
    return *this;
  }

  Segment&
  Segment::
  operator/= (const double& a)
  {
// TODO: Gestire il caso a=0;
    p1 /= a;
    p2 /= a;
    length /= a;
    return *this;
  }

  Segment
  Segment::
  operator* (const double& a) const
  {
    return Segment(*this).operator *= (a);
  }

  Segment
  Segment::
  operator/ (const double& a)
  {
    return Segment(*this).operator *= (a);
  }

  double
  get_angle(const Segment& a, const Segment& b)
  {
    auto da = a.get_direction();
    auto db = b.get_direction();
    return std::acos(scalar_product(da,db));
  }

  std::pair<Point, Point>
  closest_points(const Segment& a, const Segment& b)
  {
    int sol = 0;

    auto pa = a.get_extreme_points();
    auto pb = b.get_extreme_points();
    double min = points_distance(pa.first,  pb.first);

    double min1 = points_distance(pa.first, pb.second);
    if(min1 < min)
      {
        sol = 1;
        min = min1;
      }

    double min2 = points_distance(pa.second, pb.first);
    if(min2 < min)
      {
        sol = 2;
        min = min2;
      }

    double min3 = points_distance(pa.second, pb.second);
    if(min3 < min)
      {
        sol = 3;
        min = min3;
      }

    if(sol == 0)
      {
        return std::make_pair(pa.first,  pb.first);
      }
    else if(sol == 1)
      {
        return std::make_pair(pa.first, pb.second);
      }
    else if(sol == 2)
      {
        return std::make_pair(pa.second, pb.first);
      }
    else
      {
        return std::make_pair(pa.second, pb.second);
      }

  }

  double
  min_distance(const Segment& a, const Segment& b)
  {
    std::pair<Point, Point> p = closest_points(a, b);
    return points_distance(p.first, p.second);
  }

  STATUS
  are_aligned(const Segment& a, const Segment& b, const double& tolerance)
  {
    std::pair<Point, Point> c_p = closest_points(a,b);
    Segment c(c_p.first, c_p.second);
    double angle_ac = M_PI / 2.0 - std::abs(M_PI / 2.0 - get_angle(a, c));
    double angle_cb = M_PI / 2.0 - std::abs(M_PI / 2.0 - get_angle(b, c));

    if(angle_ac < tolerance && angle_cb < tolerance)
      {
        return STATUS::MLIB_SUCCEED;
      }
    else
      {
        return STATUS::MLIB_ERROR;
      }
  }

}
