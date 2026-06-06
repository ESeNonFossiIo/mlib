#ifndef _NUMERIX__CLOTHOID__H_
#define _NUMERIX__CLOTHOID__H_

#include "numerix/math/integral.h"
#include "numerix/math/point.h"

/** \addtogroup math
 *  @{
 */

namespace numerix {

class Clothoid {
public:
    Clothoid(const double& a_, const Integral& integral_ = Integral(TrapezoidalRule(), 0.01));

    /**
     * Evaluate the clothoid in x
     */
    Point operator()(const double& x) const;

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

class ApproximatedClothoid {
public:
    ApproximatedClothoid(const double& a_);

    /**
     * Evaluate the clothoid in x
     */
    Point operator()(const double& x) const;

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

} // namespace numerix

/** @}*/
#endif //_NUMERIX__CLOTHOID__H_
