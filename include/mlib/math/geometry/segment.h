#ifndef _MLIB__SEGMENT__H_
#define _MLIB__SEGMENT__H_

#include "mlib/math/point.h"

#include <mlib/utility/status.h>

/** \addtogroup math
 *  @{
 */

namespace mlib
{

  class Segment
  {
  public:
    Segment();

    Segment(const Point& p1_, const Point& p2_);

    void
    update(const Point& p1_, const Point& p2_);

    Point
    get_direction() const;

    double
    get_length() const;

    std::pair<Point, Point>
    get_extreme_points() const;

    /**
     *
     */
    Segment& operator*= (const double& a);

    /**
     *
     */
    Segment& operator/= (const double& a);

    /**
     *
     */
    Segment operator* (const double& a) const;

    /**
     *
     */
    Segment operator/ (const double& a);

  private:
    Point p1;
    Point p2;
    double length;
    Point d;
  };

  double
  get_angle(const Segment& a, const Segment& b);

  std::pair<Point, Point>
  closest_points(const Segment& a, const Segment& b);

  double
  min_distance(const Segment& a, const Segment& b);

  STATUS
  are_aligned(const Segment& a, const Segment& b, const double& tolerance = 0.01);

};

/** @}*/
#endif //_MLIB__SEGMENT__H_
