#ifndef __m_POINT_H__
#define __m_POINT_H__

#include "mlib/math/matrix/matrix.h"

#include <assert.h>     /* assert */
#include <cmath>     /* pow */
#include <tuple>

#ifdef MLIB_USE_PCL
#include <pcl/point_types.h>
#endif //MLIB_USE_PCL

/** \addtogroup math
 *  @{
 */
namespace mlib
{
  /**
   * @brief The Point class
   */
  class Point : public Matrix<double>
  {
  public:
    /**
     * @brief constructor
     */
    Point(bool normalize = false);

    /**
     * @brief constructor
     */
    Point(std::initializer_list<double> list,
          bool normalize = false);


    /**
     * @brief Point
     * @param x_
     */
    Point(double x_);

    /**
     * @brief Point
     * @param x_
     * @param y_
     * @param z_
     */
    Point(double x_, double y_);

    /**
     * @brief Point
     * @param x_
     * @param y_
     * @param z_
     */
    Point(double x_, double y_, double z_);

    /**
     * @brief Point
     * @param x_
     * @param y_
     * @param z_
     * @param t_
     */
    Point(double x_, double y_, double z_, double t_);

    /**
     * @brief Copy constructor.
     * @param q point to copy.
     */
    Point(const Point& q);

#ifdef MLIB_USE_PCL
    Point(const pcl::PointXYZ& p);

    Point(const pcl::PointXYZI& p);

    // Point& Operator= (const pcl::PointXYZ& p);
    //
    // Point& Operator= (const pcl::PointXYZI& p);
#endif //MLIB_USE_PCL

    void resize(const size_t& n);

    /**
     *
     */
    Point(const Matrix<double>& M);

    double&
    operator[](size_t i);

    const double&
    operator[](size_t i) const;

    double&
    operator()(size_t i);

    const double&
    operator()(size_t i) const;

    /**
     * @brief x
     * @return
     */
    const double&
    x() const;

    /**
     * @brief y
     * @return
     */
    const double&
    y() const;

    /**
     * @brief z
     * @return
     */
    const double&
    z() const;

    /**
     * @brief z
     * @return
     */
    const double&
    w() const;

    /**
     * @brief norm
     * @return
     */
    double norm() const;

    /**
     * @brief normalize
     * @return
     */
    void normalize();

    Point
    operator= (const Matrix<double> M);

    size_t dim() const;

    bool is_normalized;
  };

  /**
   * @brief operator ^
   * @param p1
   * @param p2
   * @return
   */
  Point
  operator^ (const Point& p1, const Point& p2);

  /**
   * @brief operator ^
   * @param p1
   * @param p2
   * @return
   */
  double
  vector_product(const Point& p1, const Point& p2);

  /**
   * @brief scalar_product
   * @param p1
   * @param p2
   * @return
   */
  double
  scalar_product(const Point& p1, const Point& p2);

  /**
   * @brief operator ^
   * @param p1
   * @param p2
   * @return
   */
  double
  points_distance(const Point& p1,
                  const Point& p2,
                  const unsigned int& d = 2);

  /**
   * [Pointcentroid description]
   * @param p [description]
   * @param w [description]
   */
  Point
  centroid(const std::vector<Point>& p,
           const std::vector<double>& w);
};

/** @}*/
#endif // __m_POINT_H__
