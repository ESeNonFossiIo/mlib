#ifdef MLIB_USE_EIGEN3

#ifndef _MLIB_EIGEN_CONVERSION_
#define _MLIB_EIGEN_CONVERSION_

#include "mlib/math/matrix/matrix.h"

#include <iostream>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

/** \addtogroup math
 *  @{
 */

namespace mlib
{

  /**
   * [from_eigen_to_m_matrix description]
   * @param  m [description]
   * @return   [description]
   */
  Eigen::MatrixXd
  from_m_to_eigen_matrix(const Matrixd& m)
  {
    Eigen::MatrixXd m_new(m.r(),m.c());
    for(unsigned int i = 0; i < m.r(); i++)
      for(unsigned int j = 0; j < m.c(); j++)
        m_new(i,j) = m(i,j);
    return m_new;
  }

  /**
   * [from_m_to_eigen_matrix description]
   * @param  m [description]
   * @return   [description]
   */
  Matrixd
  from_eigen_to_m_matrix(const Eigen::MatrixXd& m)
  {
    Matrixd m_new(m.rows(), m.cols());
    for(unsigned int i = 0; i < m.rows(); i++)
      for(unsigned int j = 0; j < m.cols(); j++)
        m_new(i,j) = m(i,j);
    return m_new;
  }

};


/** @}*/

#endif //_MLIB_EIGEN_CONVERSION_

#endif //MLIB_USE_EIGEN3
