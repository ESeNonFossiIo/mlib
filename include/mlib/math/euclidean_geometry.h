#ifndef __EUCLIDEAN_GEOMETRY_H__
#define __EUCLIDEAN_GEOMETRY_H__

#include <mlib/math/point.h>
#include "mlib/math/matrix/matrix.h"

/** \addtogroup math
 *  @{
 */
namespace mlib
{
  class HyperPlane : public Point
  {
  public:
    /**
     * [HyperPlane description]
     */
    HyperPlane();

    /**
     * [HyperPlane description]
     * @param list        [description]
     * @param normal_form [description]
     */
    HyperPlane(std::initializer_list<double> list,
               bool normal_form = false);

    /**
     * [normal_form description]
     */
    void normal_form();

    /**
     * @brief a
     * @return
     */
    double a() const;

    /**
     * @brief b
     * @return
     */
    double b() const;

    /**
     * @brief c
     * @return
     */
    double c() const;

    /**
     * @brief d
     * @return
     */
    double d() const;
  };

  /**
   * [middle_hyperplane_between_points description]
   * @param  p [description]
   * @param  q [description]
   * @return   [description]
   */
  HyperPlane
  middle_hyperplane_between_points(const Point& p,
                                   const Point& q);

  /**
   * [get_hyperplanes_intersection description]
   * @param  p [description]
   * @param  q [description]
   * @return   [description]
   */
  Point
  get_hyperplanes_intersection(const HyperPlane& p,
                               const HyperPlane& q);

  /**
   * [get_hyperplanes_intersection description]
   * @param  p [description]
   * @param  q [description]
   * @param  r [description]
   * @return   [description]
   */
  Point
  get_hyperplanes_intersection(const HyperPlane& p,
                               const HyperPlane& q,
                               const HyperPlane& r);
  /**
   * [circumference_center description]
   * @param  a [description]
   * @param  b [description]
   * @param  c [description]
   * @return   [description]
   */
  Point
  circumference_center(const Point& a,
                       const Point& b,
                       const Point& c);

  /**
   * [hyperplane_passing_through_three_points description]
   * @param  a [description]
   * @param  b [description]
   * @param  c [description]
   * @return   [description]
   */
  HyperPlane
  hyperplane_passing_through_three_points(const Point& a,
                                          const Point& b,
                                          const Point& c);
  /**
   * [circumference_center description]
   * @param  a [description]
   * @param  b [description]
   * @param  c [description]
   * @return   [description]
   */
  Point
  circumference_center(const Point& a,
                       const Point& b,
                       const Point& c);

}

/** @}*/
#endif // __EUCLIDEAN_GEOMETRY_H__
