#ifndef  __m_ROTOTRANSLATION_MATRIX_H__
#define  __m_ROTOTRANSLATION_MATRIX_H__

#include "mlib/math/matrix/matrix.h"
#include "mlib/math/angle.h"
#include "mlib/math/point.h"

#include <math.h>       /* atan2 */

/** \addtogroup math
 *  @{
 */

namespace _mlib
{
  /**
   *  Invert line l1 and l2
   */
  void swap_lines(Matrix<double>& M,
                  const unsigned int& l1,
                  const unsigned int& l2);

  /**
   *  Invert line l1 and l2 in M and b
   */
  void swap_lines(Matrix<double>& M,
                  Point& b,
                  const unsigned int& l1,
                  const unsigned int& l2);

  /**
   *  Calculate the equavalent upper triangular system
   */
  std::pair<Matrix<double>, Point>
  upper_triangular(const Matrix<double>& M, const Point& b);

  /**
   * Solve an upper triangular system
   */
  Point
  solve_upper_triangular(const Matrix<double>& M, const Point& b);

  /**
   * Solve a system using the gauss elimination algorithm
   */
  Point
  gauss_elimination_upper(const Matrix<double>& M, const Point& b);


  /**
   * Get the best (in terms of L2 norm) matrix Mx + b that link p to q points.
   * Size of p and q have to be greater than 4.cm,
   */
  double
  best_affine_transformation(const std::vector<Point>& p,
                             const std::vector<Point>& q,
                             Matrix<double>& M,
                             Point& b);

  /**
   * Get the RotoTranslation matrix Mx + b that link p to q points.
   * Size of p and q have to be greater than 4.cm,
   */
  double
  rototranslation_matrix(const std::vector<Point>& p,
                         const std::vector<Point>& q,
                         Matrix<double>& M,
                         Point& b);

  /**
   * Get the RotoTranslation matrix M(x-q1) + q1 + b that link p1,p2 to q1,q2 points.
   */
  double
  rototranslation_matrix_2d(const Point& p1, const Point& p2,
                            const Point& q1, const Point& q2,
                            Matrix<double>& M,
                            Point& t);

  class RotoTranslationMatrixHandler
  {
  public:
    /**
     * RotoTranslationMatrixHandler constructor
     * @param u_ initial points
     * @param v_ final points
     */
    RotoTranslationMatrixHandler(const std::vector<Point>& u_,
                                 const std::vector<Point>& v_);

    /**
     * Define a new boolean mask to select points
     * @param b_ boolean mask
     */
    void update_mask(const std::vector<size_t>& b_);

    /**
     * return errror on the i-nth point.
     * @param  i index of the point
     * @return   error
     */
    double get_error_on_point(const size_t& i);

    /**
     * return the mean error evaluated using the euclidean distance
     * @return error
     */
    double get_error();

    /**
     * Return the difference of volume in the transformation
     * @return variation
     */
    double get_volume_diff();

    /**
     * retrun the number of point used in the
     * @return the number of points that have been used to calculate the matrix and the vector
     */
    size_t get_number_of_points();

    /**
     * Return the Rotation/Dilatation Matrix
     * @return matrix
     */
    Matrix<double>& get_matrix();

    /**
     * Return the translation vector
     * @return Point that represent the translation
     */
    Point& get_translation();

    /**
     * Boolean mask
     */
    std::vector<size_t> b;

  private:
    /**
     * Rotation/Dilatation Matrix
     */
    Matrix<double> M;
    /**
     * Translation vector
     */
    Point t;
    /**
     * Total error
     */
    double error;
    /**
     * Number of used points
     */
    size_t np;

    /**
     * Initial configuration
     */
    const std::vector<Point> u;
    /**
     * Final configuration
     */
    const std::vector<Point> v;
  };
}


/** @}*/
#endif // __m_ROTOTRANSLATION_MATRIX_H__
