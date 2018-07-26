#include "mlib/math/angle.h"
#include "mlib/math/matrix/rotation.h"

#include <cassert>

/// Angle
////////////////////////////////////////////////////////////////////////////////

namespace mlib
{

  Angle::
  Angle()
  {
    angleType = AngleType::rad;
    angleRad = 0;
    angleDeg = 0;
  }

  Angle::
  Angle(const double& _angle,
        const AngleType& _angleType)
  {
    angleType = _angleType;
    if(angleType == AngleType::rad)
      {
        angleRad = _angle;
        angleDeg = (180.0/M_PI) *angleRad;
      }
    else
      {
        angleDeg = _angle;
        angleRad = (M_PI/180.0) *angleDeg;
      }
  }

  Angle::
  Angle(const Angle& copyAngle)
  {
    angleDeg   = copyAngle.deg();
    angleRad   = copyAngle.rad();
    angleType  = copyAngle.type();
  }

  Angle
  Angle::
  operator= (const Angle& copyAngle) const
  {
    angleDeg = copyAngle.deg();
    angleRad = copyAngle.rad();
    angleType = copyAngle.type();
    return *this;
  }

  Angle
  Angle::
  operator= (const double& val) const
  {
    if(angleType == AngleType::rad)
      {
        angleRad = val;
        angleDeg = (180.0/M_PI) *angleRad;
      }
    else
      {
        angleDeg = val;
        angleRad = (M_PI/180.0) *angleDeg;
      }
    return *this;
  }

  Angle&
  Angle::
  operator*= (const double& a)
  {
    angleRad *= a;
    angleDeg *= a;
    return *this;
  };

  double
  Angle::
  deg() const
  {
    return this->angleDeg;
  }

  double
  Angle::
  rad() const
  {
    return this->angleRad;
  }

  AngleType
  Angle::
  type() const
  {
    return this->angleType;
  }


/// TaitBryanAngles
////////////////////////////////////////////////////////////////////////////////

  TaitBryanAngles::
  TaitBryanAngles()
  {}


  TaitBryanAngles::
  TaitBryanAngles(
    const Angle& _roll,
    const Angle& _pitch,
    const Angle& _yaw)
    :
    rollAngle(_roll),
    pitchAngle(_pitch),
    yawAngle(_yaw)
  {}

  TaitBryanAngles::
  TaitBryanAngles(
    const double& _roll,
    const double& _pitch,
    const double& _yaw,
    const AngleType& angle_type)
    :
    rollAngle(Angle(_roll, angle_type)),
    pitchAngle(Angle(_pitch, angle_type)),
    yawAngle(Angle(_yaw, angle_type))
  {}

  double
  TaitBryanAngles::
  roll(AngleType angleType) const
  {
    if(angleType == AngleType::rad)
      return this->rollAngle.rad();
    else
      return this->rollAngle.deg();
  }


  double
  TaitBryanAngles::
  pitch(AngleType angleType) const
  {
    if(angleType == AngleType::rad)
      return pitchAngle.rad();
    else
      return pitchAngle.deg();
  }

  double
  TaitBryanAngles::
  yaw(AngleType angleType) const
  {
    if(angleType == AngleType::rad)
      return yawAngle.rad();
    else
      return yawAngle.deg();
  }

  void
  TaitBryanAngles::
  roll(const double& angle,
       AngleType angleType) const
  {
    Angle tmp(angle, angleType);
    rollAngle = tmp;
  }

  void
  TaitBryanAngles::
  pitch(const double& angle,
        AngleType angleType) const
  {
    pitchAngle = Angle(angle, angleType);
  }

  void
  TaitBryanAngles::
  yaw(const double& angle,
      AngleType angleType) const
  {
    yawAngle = Angle(angle, angleType);
  }

/// TaitBryanAngles
////////////////////////////////////////////////////////////////////////////////

  Point
  convert_angle_to_vector(const Angle& angle,
                          const Point& axis,
                          const bool& clockwise)
  {
    Angle rotation_angle(angle);
    if(clockwise)
      rotation_angle *= -1;
    Rotation2DMatrix R(rotation_angle);
    Point r(R*axis);
    r.normalize();
    return r;
  }

  Angle
  convert_vector_to_angle(const Point& direction,
                          const Point& axis,
                          const bool& clockwise)
  {
    Point d(direction);
    Point a(axis);
    d.normalize();
    a.normalize();

    double angle = std::acos(d.x()*a.x() + d.y()*a.y());
    double s = d.x()*a.y() - d.y()*a.x();
    if(s<0)
      {
        angle *=-1.0;
      }

    if(!clockwise)
      {
        angle *= -1.0;
      }

    return Angle(angle);
  }

  Angle
  get_angle_from_points(const Point& a,
                        const Point& o,
                        const Point& b)
  {
    assert((a-o).l_2_norm()*(b-o).l_2_norm() > 0);
    double angle = std::acos(((a-o).t()*(b-o))[0]/((a-o).l_2_norm()*
                                                   (b-o).l_2_norm()));
    return Angle(angle);
  };
}
