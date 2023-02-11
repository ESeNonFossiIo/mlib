#ifndef __CONSTANTS__H_
#define __CONSTANTS__H_

/** \addtogroup math
 *  @{
 */

namespace mlib
{

  namespace constants
  {

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795028841971
#endif //__M_PI__H_

/// sqrt(2 * pi)
    constexpr double sqrt_two_pi = 2.50662827463100050241576528481104525300698674;

/// 1 / sqrt(2 * pi)
    constexpr double inv_sqrt_two_pi =
      0.3989422804014326779399460599343818684758586;
  }

}

/** @}*/
#endif //__CONSTANTS__H_
