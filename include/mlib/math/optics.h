#ifndef __OPTICS__H_
#define __OPTICS__H_

#include "mlib/math/matrix/matrix.h"

/** \addtogroup math
 *  @{
 */

namespace mlib
{

  class CameraMatrix
  {
    CameraMatrix()
      :
      K(3,3),
      R(3,3),
      T(3,1)
    {

    }

    /**
     * skew coefficient between the x and the y axis
     */
    double gamma()
    {
      return m(0,2);
    }

    /**
     * focal length in terms of pixels
     */
    double px_f_x()
    {
      return m(0,0);
    }

    /**
     * focal length in terms of pixels
     */
    double px_f_y()
    {
      return m(1,1);
    }

    /**
     * focal length in terms of pixels
     */
    double p_p_x()
    {
      return m(0,2);
    }

    /**
     * focal length in terms of pixels
     */
    double p_p_y()
    {
      return m(1,2);
    }

    Matrixd K;
    Matrixd R;
    Matrixd T;
  };

}

/** @}*/
#endif //__OPTICS__H_
