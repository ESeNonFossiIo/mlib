#include "mlib/math/matrix/rotation.h"
#include <cmath>

namespace mlib
{

  std::pair<Point, Angle>
  get_axis_and_angle(const Matrixd& m)
  {
    double theta = acos(0.5 * (m.trace() - 1));
    Point axis;
    axis.resize(3);
    if(std::abs(sin(theta)) > VAR_MLIB_ZERO_TOLERANCE)
      {
        axis[0] = (m(2,1) - m(1,2)) / (2 * sin(theta));
        axis[1] = (m(0,2) - m(2,0)) / (2 * sin(theta));
        axis[2] = (m(1,0) - m(0,1)) / (2 * sin(theta));
      }
    return std::make_pair(axis, Angle(theta));
  }

  Rotation2DMatrix::
  Rotation2DMatrix(Angle theta_)
    :
    Matrix<double> (2,2),
    theta(theta_)
  {
    this->elements[0] = std::cos(theta.rad());
    this->elements[2] = std::sin(theta.rad());
    this->elements[1] = -std::sin(theta.rad());
    this->elements[3] = std::cos(theta.rad());
  }

  RotationMatrix::
  RotationMatrix(const Point& rodriguez_vector_)
    :
    Matrix<double> (3,3)
  {
    double theta = rodriguez_vector_.norm();
    if(theta != 0)
      {
        Point u(rodriguez_vector_);
        u /= theta;

        Matrix<double> M(3,3);
        Matrixd I({{1,0,0},{0,1,0},{0,0,1}});
        Matrixd A({ {0,    -u[2],   u[1]},
          {u[2],     0,  -u[0]},
          {-u[1], u[0],     0}
        });

        M = std::cos(theta) * I +
            (1-std::cos(theta)) * (u*u.t()) +
            std::sin(theta) * A;

        for(unsigned int i = 0; i<9; ++i)
          this->elements[i] = M[i];
      }
  }

  RotationMatrix::
  RotationMatrix(Point axis_,Angle theta_)
    :
    Matrix<double> (3,3),
    axis(axis_),
    theta(theta_)
  {
    Matrix<double> M(
    {
      {0,           -axis.z(),    axis.y() },
      {axis.z(),    0,            -axis.x() },
      {-axis.y(),   axis.x(),     0}
    });
    IdentityMatrix id(3);

    Matrix<double> Matrix = std::sin(theta.rad()) *M + std::cos(
                              theta.rad()) * (id - axis*axis.t()) + axis*axis.t();
    for(unsigned int i = 0; i<9; ++i)
      this->elements[i] = Matrix[i];
  }

  RotationMatrix::
  RotationMatrix(Matrix axis_,Angle theta_)
    :
    Matrix<double> (3,3),
    axis(
  {
    axis_[0],axis_[1],axis_[2]
  }),
  theta(theta_)
  {
    Matrix<double> M(
    {
      {0,           -axis.z(),    axis.y() },
      {axis.z(),    0,            -axis.x() },
      {-axis.y(),   axis.x(),     0}
    });
    IdentityMatrix id(3);

    Matrix<double> Matrix = std::sin(theta.rad()) *M + std::cos(
                              theta.rad()) * (id - axis*axis.t()) + axis*axis.t();
    for(unsigned int i = 0; i<9; ++i)
      this->elements[i] = Matrix[i];
  }

  RotationMatrix::
  RotationMatrix(TaitBryanAngles angles, RotationType type)
    :
    Matrix<double> (type == RotationType::XYZ ?
                    RotationMatrix(Point(1,0,0), angles.roll()) *
                    RotationMatrix(Point(0,1,0), angles.pitch()) *
                    RotationMatrix(Point(0,0,1), angles.yaw())
                    :
                    RotationMatrix(Point(0,0,1), angles.yaw()) *
                    RotationMatrix(Point(0,1,0), angles.pitch()) *
                    RotationMatrix(Point(1,0,0), angles.roll()))

  {
    std::pair<Point, Angle> result = get_axis_and_angle(
                                       *this);
    axis  = result.first;
    theta = result.second;
  }

  TaitBryanAngles
  RotationMatrix::
  get_angles(const RotationType type,
             const double& defaul_yaw)
  {
    return get_roll_pitch_yaw(*this, type, defaul_yaw);
  }

  TaitBryanAngles
  get_roll_pitch_yaw(const Matrixd& m,
                     const RotationType type,
                     const double& defaul_yaw)
  {
    Angle roll(0);
    Angle yaw(0);
    Angle pitch(0);

    if(type == RotationType::ZYX)
      {
        if(std::abs(m(2,0)) < 1  - VAR_MLIB_ZERO_TOLERANCE)
          {
            pitch = asin(-m(2,0));
            roll = atan2(m(2,1) /cos(pitch.rad()), m(2, 2) /cos(pitch.rad()));
            yaw = atan2(m(1,0) /cos(pitch.rad()), m(0, 0) /cos(pitch.rad()));
          }
        else
          {
            if(m(2,0) > 0)
              {
                yaw   = defaul_yaw; // anything
                pitch = - M_PI / 2.0;
                roll  = atan2(-m(0,1), -m(0,2)) - yaw.rad();
              }
            else
              {
                yaw   = defaul_yaw; // anything
                pitch = M_PI / 2.0;
                roll  = atan2(m(0,1), m(0,2)) + yaw.rad();
              }
          }
      }
    else
      {
        if(std::abs(m(0,2)) < 1 - VAR_MLIB_ZERO_TOLERANCE)
          {
            pitch = asin(m(0,2));
            roll  = atan2(-m(1,2) /cos(pitch.rad()), m(2,2) /cos(pitch.rad()));
            yaw   = atan2(-m(0,1) /cos(pitch.rad()), m(0, 0) /cos(pitch.rad()));

          }
        else
          {
            if(m(0,2) > 0)
              {
                yaw   = defaul_yaw; // anything
                pitch = M_PI / 2.0;
                roll  = M_PI -  atan2(m(1,0), m(2,0)) - yaw.rad();
              }
            else
              {
                yaw   = defaul_yaw; // anything
                pitch = - M_PI / 2.0;
                roll  = - atan2(m(1,0), m(2,0)) + yaw.rad();
              }
          }
      }
    return TaitBryanAngles(roll, pitch, yaw);
  }

}
