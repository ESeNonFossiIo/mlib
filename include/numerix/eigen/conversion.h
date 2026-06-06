#ifdef NUMERIX_USE_EIGEN3

#ifndef _NUMERIX_EIGEN_CONVERSION_
#define _NUMERIX_EIGEN_CONVERSION_

#include "numerix/math/matrix/matrix.h"

#include <iostream>

// Silence warnings inside Eigen headers (deprecated implicit copy/dtor,
// sign comparisons, ...). They are not actionable from numerix code.
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif
#if defined(__clang__)
#pragma GCC diagnostic ignored "-Wdeprecated-copy-with-dtor"
#endif

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

/** \addtogroup math
 *  @{
 */

namespace numerix {

/**
 * [from_eigen_to_m_matrix description]
 * @param  m [description]
 * @return   [description]
 */
Eigen::MatrixXd from_m_to_eigen_matrix(const Matrixd& m)
{
    Eigen::MatrixXd m_new(m.r(), m.c());
    for (std::size_t i = 0; i < m.r(); i++)
        for (std::size_t j = 0; j < m.c(); j++)
            m_new(i, j) = m(i, j);
    return m_new;
}

/**
 * [from_m_to_eigen_matrix description]
 * @param  m [description]
 * @return   [description]
 */
Matrixd from_eigen_to_m_matrix(const Eigen::MatrixXd& m)
{
    Matrixd m_new(m.rows(), m.cols());
    // m.rows()/m.cols() return Eigen::Index (signed) — compare with the same
    // type to avoid -Wsign-compare under -Werror.
    for (Eigen::Index i = 0; i < m.rows(); i++)
        for (Eigen::Index j = 0; j < m.cols(); j++)
            m_new(i, j) = m(i, j);
    return m_new;
}

}; // namespace numerix

/** @}*/

#endif //_NUMERIX_EIGEN_CONVERSION_

#endif // NUMERIX_USE_EIGEN3
