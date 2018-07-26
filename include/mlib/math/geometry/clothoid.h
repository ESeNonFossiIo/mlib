#ifndef _MLIB__CLOTHOID__H_
#define _MLIB__CLOTHOID__H_

#include "mlib/math/point.h"
#include "mlib/math/integral.h"

/** \addtogroup math
 *  @{
 */

namespace mlib
{

  class Clothoid
  {
  public:
    Clothoid(const double& a_,
             const Integral& integral_ = Integral(TrapezoidalRule(), 0.01));

    /**
     * Evaluate the clothoid in x
     */
    Point
    operator()(const double& x) const;

    /**
     * Clothoid curvature
     */
    double k(const double& x) const;

    /**
     * Clothoid length
     */
    double l(const double& x) const;

    /**
     * Clothoid param
     */
    double a;
    Integral integral;
  };

  class ApproximatedClothoid
  {
  public:
    ApproximatedClothoid(const double& a_);

    /**
     * Evaluate the clothoid in x
     */
    Point
    operator()(const double& x) const;

    /**
     * Clothoid length
     */
    double l(const double& x) const;

    /**
     * Clothoid curvature
     */
    double k(const double& x) const;

    /**
     * Clothoid param
     */
    double a;
  };

};

/** @}*/
#endif //_MLIB__CLOTHOID__H_
