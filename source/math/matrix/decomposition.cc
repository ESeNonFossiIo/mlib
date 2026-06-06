#include <numerix/core/unused.h>
#include <numerix/math/matrix/decomposition.h>
#include <numerix/math/point.h>

#ifdef NUMERIX_USE_EIGEN3
#include "numerix/eigen/conversion.h"
#endif // NUMERIX_USE_EIGEN3
#
#include <cassert>
#include <cmath>

namespace numerix {

void SVD(const Matrixd& A, Matrixd& U, Matrixd& W, Matrixd& V)
{
#ifdef NUMERIX_USE_EIGEN3
    // Eigen JacobiSVD works fine with square matrix. In the case of rectangular
    // matrix you can get only thin U and V.
    assert(A.r() == A.c());

    Eigen::MatrixXd a = from_m_to_eigen_matrix(A);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeFullU | Eigen::ComputeFullV);

    size_t m = A.r();
    size_t n = A.c();

    U.resize(m, n);
    U = from_eigen_to_m_matrix(svd.matrixU());

    V.resize(m, n);
    V = from_eigen_to_m_matrix(svd.matrixV());

    W.resize(m, n);
    for (std::size_t i = 0; i < A.r(); ++i) {
        W(i, i) = svd.singularValues()(i);
    }

#else  // NUMERIX_USE_EIGEN3
    // TODO
    NUMERIX_UNUSED(A);
    NUMERIX_UNUSED(U);
    NUMERIX_UNUSED(W);
    NUMERIX_UNUSED(V);
    assert(true);
#endif // NUMERIX_USE_EIGEN3
}

void QR(const Matrixd& A, Matrixd& Q, Matrixd& R)
{
    size_t d = A.c();
    assert(A.r() == d);
    Q.resize(d, d);
    R.resize(d, d);

    for (size_t j = 0; j < d; ++j) {
        Matrixd e = A.c(j);
        for (size_t i = 0; i < j; ++i)
            e -= (Q.c(i).t() * e)[0] * Q.c(i);
        e /= e.l_2_norm();
        for (size_t i = 0; i < d; ++i)
            Q(i, j) = e[i];
    }

    for (size_t j = 0; j < d; ++j)
        for (size_t i = 0; i <= j; ++i)
            R(i, j) = (Q.c(i).t() * A.c(j))[0];
}
} // namespace numerix
