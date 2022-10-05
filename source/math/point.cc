#include "mlib/math/point.h"

// std::sqrt
#include <cmath>

namespace mlib
{
  Point::
  Point(bool normalize)
    :
    Matrix<double> (1, 1),
    is_normalized(normalize)
  {
    if(is_normalized)
      this->normalize();
  }

  Point::
  Point(std::initializer_list<double> list,
        bool normalize)
    :
    Matrix<double> (Matrix<double> (
  {
    list
  }).t()),
  is_normalized(normalize)
  {
    if(is_normalized)
      this->normalize();
  }

  Point::
  Point(double x_)
    :
    Matrix<double> (1, 1)
  {
    elements[0] = x_;
  }

  Point::
  Point(double x_, double y_)
    :
    Point(
  {
    x_,y_
  })
  {}

  Point::
  Point(double x_, double y_, double z_)
    :
    Point(
  {
    x_,y_,z_
  })
  {}

  Point::
  Point(double x_, double y_, double z_, double t_)
    :
    Point(
  {
    x_,y_,z_,t_
  })
  {}

#ifdef MLIB_USE_PCL
  Point::
  Point(const pcl::PointXYZ& p)
    :
    Point(p.x,p.y,p.z)
  {}

  Point::
  Point(const pcl::PointXYZI& p)
    :
    Point(p.x,p.y,p.z)
  {}

  // Point&
  // Point::
  // Operator= (const pcl::PointXYZ& p)
  // {
  //   this->reshape(3);
  //   this->elements[0]=p.x;
  //   this->elements[1]=p.y;
  //   this->elements[2]=p.z;
  //   return *this;
  // }
  //
  // Point&
  // Point::
  // Operator= (const pcl::PointXYZI& p)
  // {
  //   this->reshape(3);
  //   this->elements[0]=p.x;
  //   this->elements[1]=p.y;
  //   this->elements[2]=p.z;
  //   return *this;
  // }

#endif //MLIB_USE_PCL

  Point::
  Point(const Matrix<double>& M)
    :
    Matrix<double> (M) {}

  void
  Point::
  resize(const size_t& n)
  {
    this->rows = n;
    this->cols = 1;
    this->elements.resize(n);
  }

  double&
  Point::
  operator[](size_t i)
  {
    return elements[i];
  }

  const double&
  Point::
  operator[](size_t i) const
  {
    return elements[i];
  }

  double&
  Point::
  operator()(size_t i)
  {
    return elements[i];
  }

  const double&
  Point::
  operator()(size_t i) const
  {
    return elements[i];
  }

  const double&
  Point::
  x() const
  {
    assert(this->r()>=1);
    return elements[0];
  }

  const double&
  Point::
  y() const
  {
    assert(this->r()>=2);
    return elements[1];
  }

  const double&
  Point::
  z() const
  {
    assert(this->r()>=3);
    return elements[2];
  }

  const double&
  Point::
  w() const
  {
    assert(this->r()>=4);
    return elements[3];
  }

  double
  Point::
  norm() const
  {
    double sum = 0.0;
    for(std::size_t i = 0; i < this->dim(); ++i)
      sum += elements[i]*elements[i];
    return std::sqrt(sum);
  }

  void
  Point::
  normalize()
  {
    double norm = this->norm();
    assert(norm>0);

    for(size_t i = 0; i < this->dim(); ++i)
      this->elements[i]/= norm;
  }

  Point
  Point::
  operator= (const Matrix<double> M)
  {
    this->resize(M.r());
    for(size_t i = 0; i < this->dim(); ++i)
      this->elements[i] = M(i);
    return *this;
  }

  size_t
  Point::
  dim() const
  {
    return this->r();
  }

  Point
  operator^ (const Point& p1, const Point& p2)
  {
    assert(p1.dim() == 3);
    assert(p2.dim() == 3);

    return Point(p1.y() * p2.z() - p1.z() *
                 p2.y(),   //a_y b_z - a_z b_y
                 p1.z() * p2.x() - p1.x() * p2.z(),  //a_z b_x - a_x b_z
                 p1.x() * p2.y() - p1.y() * p2.x());  //a_x b_y - a_y b_x
  }

  double
  scalar_product(const Point& p1, const Point& p2)
  {
    return (p1.t() * p2)[0];
  }

  double
  vector_product(const Point& p1, const Point& p2)
  {
    assert(p1.dim() == 2);
    assert(p2.dim() == 2);

    return p1.x()*p2.y() - p2.x()*p1.y();
  }

  double
  points_distance(const Point& p1,
                  const Point& p2,
                  const std::size_t& d)
  {
    double sum = 0;
    for(std::size_t i = 0; i < p1.dim(); i++)
      {
        sum += std::pow(std::abs(p1[i] - p2[i]), d);
      }
    return std::pow(sum, 1.0/(float)d);
  }

  Point
  centroid(const std::vector<Point>& p,
           const std::vector<double>& w)
  {
    assert(p.size() == w.size());

    Point sum_p(0 * p[0]);
    double sum_w = 0.0;

    for(std::size_t i = 0; i < p.size(); ++i)
      {
        sum_p += (w[i] * p[i]);
        sum_w += w[i];
      }
    assert(sum_w > 0);

    return (1.0/sum_w)*sum_p;
  }

}
