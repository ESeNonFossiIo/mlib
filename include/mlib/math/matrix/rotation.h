#ifndef  __m_ROTATION_MATRIX_H__
#define  __m_ROTATION_MATRIX_H__

#include "mlib/math/matrix/matrix.h"
#include "mlib/math/angle.h"
#include "mlib/math/point.h"
#include "mlib/math/constants.h"

#include <math.h>       /* atan2 */

/** \addtogroup math
 *  @{
 */

namespace _mlib
{
  /**
   * @brief The RotationType enum specifica il come vengono
   * misurati gli angoli.
   */
  enum class RotationType
  {
    /**
     * Sistema di riferimento intrinsico o di Eulero.
     * i.e. Yaw * Pitch * Roll
     */
    ZYX,
    /**
     * Sistema di riferimento estrinseco o basato sugli anglodi di Tait-Bryan.
     * i.e. Roll * Pitch * Yaw.
     */
    XYZ
  };

  class RotationMatrix : public Matrix<double>
  {
  public:
    /**
     * @brief constructor
     */
    RotationMatrix(Point axis_, Angle theta_);

    /**
     * @brief Uses a Rodriguez vector to construc a RotationMatrix
     */
    RotationMatrix(const Point& rodriguez_vector_);

    /**
     * @brief constructor
     */
    RotationMatrix(Matrix<double> axis_, Angle theta_);

    /**
     * @brief constructor
     */
    RotationMatrix(TaitBryanAngles angles,
                   RotationType type = RotationType::XYZ);

    /**
     * @brief get_angles
     */
    TaitBryanAngles
    get_angles(const RotationType type = RotationType::XYZ,
               const double& defaul_yaw = M_PI / 3.0);

  private:
    /**
     *
     */
    Point axis;

    /**
     *
     */
    Angle theta;
  };

  class Rotation2DMatrix : public Matrix<double>
  {
  public:
    /**
     * @brief constructor
     */
    Rotation2DMatrix(Angle theta_);

  private:

    /**
     *
     */
    Angle theta;
  };

  /**
   * @brief
   * @return std::pair. first element is the rotation axis, the second element is
   *         the angle theta.
   */
  std::pair<Point, Angle>
  get_axis_and_angle(const Matrixd& m);

  /**
   * @brief Given a rotation matrix, this function returns its
   * roll \f[\in ]-\pi, \pi ]\f], pitch \f[\in ]-\frac{\pi}{2}, and  \frac{\pi}{2}] \f],
   * yaw \f[\in ]-\pi, \pi ]\f].
   */
  TaitBryanAngles
  get_roll_pitch_yaw(const Matrixd& m,
                     const RotationType type = RotationType::XYZ,
                     const double& defaul_yaw = M_PI / 3.0);

}
/** @}*/
#endif // __m_ROTATION_MATRIX_H__
