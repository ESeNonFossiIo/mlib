#include "mlib/math/matrix/decomposition.h"
#include "mlib/math/point.h"

#ifdef _MYLIB_USE_EIGEN3
#include "mlib/eigen/conversion.h"
#endif //_MYLIB_USE_EIGEN3
#
#include <cmath>
#include <cassert>

namespace _mlib
{

  void
  SVD(const Matrixd& A,
      Matrixd& U,
      Matrixd& W,
      Matrixd& V)
  {
#ifdef _MYLIB_USE_EIGEN3
    // Eigen JacobiSVD works fine with square matrix. In the case of rectangular
    // matrix you can get only thin U and V.
    assert(A.r() == A.c());

    Eigen::MatrixXd a = from_m_to_eigen_matrix(A);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a,
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);

    size_t m = A.r();
    size_t n = A.c();

    U.resize(m,n);
    U = from_eigen_to_m_matrix(svd.matrixU());

    V.resize(m,n);
    V = from_eigen_to_m_matrix(svd.matrixV());

    W.resize(m,n);
    for(unsigned int i = 0; i< A.r(); ++i)
      {
        W(i,i) = svd.singularValues()(i);
      }

#else //_MYLIB_USE_EIGEN3
    // TODO
    assert(true);
#endif //_MYLIB_USE_EIGEN3

  }

  void
  QR(const Matrixd& A,
     Matrixd& Q,
     Matrixd& R)
  {
    size_t d = A.c();
    assert(A.r() == d);
    Q.resize(d,d);
    R.resize(d,d);

    for(size_t j = 0; j < d; ++j)
      {
        Matrixd e = A.c(j);
        for(size_t i = 0; i < j; ++i)
          e -= (Q.c(i).t() * e)[0] * Q.c(i);
        e /= e.l_2_norm();
        for(size_t i = 0; i < d; ++i)
          Q(i,j) = e[i];
      }

    for(size_t j = 0; j < d; ++j)
      for(size_t i = 0; i <= j; ++i)
        R(i,j) = (Q.c(i).t() * A.c(j))[0];
  }
}
