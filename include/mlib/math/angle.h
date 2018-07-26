#ifndef TDT_ANGLE_H
#define TDT_ANGLE_H

#include <array>
#include <limits>
#include <cmath>
#include "mlib/math/constants.h"
#include "mlib/math/point.h"

/** \addtogroup math
 *  @{
 */
namespace _mlib
{

  /**
   * @brief The Type enum specifica il come vengono
   * misurati gli angoli.
   */
  enum class AngleType
  {
    /**
     * Radianti
     */
    rad,
    /**
     * Gradi
     */
    deg
  };

  /**
   * @brief The Angle class
   */
  class Angle
  {
  public:
    /**
     * @brief Angle
     */
    Angle();

    /**
     * @brief Angle
     * @param _angle
     * @param _angleType
     */
    Angle(const double& _angle,
          const AngleType& _angleType = AngleType::rad);

    /**
     * @brief Angle
     * @param copyAngle
     */
    Angle(const Angle& copyAngle);

    /**
     * @brief Angle::operator =
     * @param copyAngle
     * @return
     */
    Angle operator= (const Angle& copyAngle) const;

    /**
     * @brief Angle::operator = change the value of the angle according to the type
     *        specified in the constructor. If not specified it is @p AngleType::rad .
     * @param reset value
     * @return
     */
    Angle operator= (const double& value) const;

    /**
     * @brief ritorna la rappresentazione in gradi
     * dell'angolo.
     * @return angolo espresso in gradi.
     */
    double deg() const;

    /**
     * @brief ritorna la rappresentazione in radianti
     * dell'angolo.
     * @return angolo espresso in radianti.
     */
    double rad() const;

    /**
     * @brief ritorna il formato dell'angolo immesso.
     * @return tipo dell'angolo immesso.
     */
    AngleType type() const;

  public:
    /**
     *
     */
    Angle& operator*= (const double& a);

  protected:
    mutable double angleRad;
    mutable double angleDeg;
    mutable AngleType angleType;
  };


  /**
   * @brief The TaitBryanAngles class modellizza
   * una terna di angoli roll, pitch e yaw
   */
  class TaitBryanAngles
  {
  public:
    /**
     * @brief TaitBryanAngles
     */
    TaitBryanAngles();

    /**
     * @brief TaitBryanAngles
     * @param _roll
     * @param _pitch
     * @param _yaw
     */
    TaitBryanAngles(const Angle& _roll, const Angle& _pitch,
                    const Angle& _yaw);

    /**
     * @brief TaitBryanAngles
     * @param _roll
     * @param _pitch
     * @param _yaw
     */
    TaitBryanAngles(const double& _roll, const double& _pitch,
                    const double& _yaw,
                    const AngleType& angle_type = AngleType::rad);

    /**
     * @brief roll restituisce l'angolo che esprime la rotazione
     * attorno all'asse x.
     * @param angleType specifica se si vuole l'angolo espresso
     * in radianti o gradi.
     * @return angolo che esprime la rotazione
     * attorno all'asse x.
     */
    double roll(AngleType angleType = AngleType::rad) const;


    /**
     * @brief pitch restituisce l'angolo che esprime la rotazione
     * attorno all'asse y.
     * @param angleType specifica se si vuole l'angolo espresso
     * in radianti o gradi.
     * @return angolo che esprime la rotazione
     * attorno all'asse y.
     */
    double pitch(AngleType angleType = AngleType::rad) const;

    /**
     * @brief yaw restituisce l'angolo che esprime la rotazione
     * attorno all'asse z.
     * @param angleType specifica se si vuole l'angolo espresso
     * in radianti o gradi.
     * @return angolo che esprime la rotazione
     * attorno all'asse z.
     */
    double yaw(AngleType angleType = AngleType::rad) const;

    void roll(const double& angle,
              AngleType angleType = AngleType::rad) const;

    void pitch(const double& angle,
               AngleType angleType = AngleType::rad) const;

    void yaw(const double& angle,
             AngleType angleType = AngleType::rad) const;

  protected:

    /**
     * @brief rotazione attorno all'asse x.
     */
    Angle rollAngle;

    /**
     * @brief rotazione attorno all'asse y.
     */
    Angle pitchAngle;

    /**
     * @brief rotazione attorno all'asse z.
     */
    Angle yawAngle;
  };

  Point
  convert_angle_to_vector(const Angle& angle,
                          const Point& axis,
                          const bool& clockwise = true);

  Angle
  convert_vector_to_angle(const Point& direction,
                          const Point& axis,
                          const bool& clockwise = true);

  /**
   * @brief return the angle minor than pi defined by oa
   * and ob.
   */
  Angle
  get_angle_from_points(const Point& a,
                        const Point& o,
                        const Point& b);
}
/** @}*/
#endif // TDT_ANGLE_H
