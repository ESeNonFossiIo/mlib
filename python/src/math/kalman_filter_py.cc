#include <numerix/core/export.h>
#include <numerix/math/kalman_filter.h>

#include "_python/status.h"

#include <vector>

namespace {

/// ----------------------------------------------------------------------------
// Build an numerix::Matrixd from a row-major flat buffer coming from ctypes.
numerix::Matrixd to_matrix(const double* data, const NUMERIXInt rows, const NUMERIXInt cols)
{
    return numerix::Matrixd(static_cast<std::size_t>(rows), static_cast<std::size_t>(cols), data);
}

/// ----------------------------------------------------------------------------
// Copy an numerix::Matrixd back into a row-major flat output buffer.
void from_matrix(const numerix::Matrixd& m, double* out)
{
    for (std::size_t i = 0; i < m.r(); ++i)
        for (std::size_t j = 0; j < m.c(); ++j)
            out[i * m.c() + j] = m(i, j);
}

} // namespace

/// ----------------------------------------------------------------------------
// One full Kalman cycle (predict + measurement update) for a scalar/vector
// linear model. Every matrix crosses the boundary as a row-major double buffer
// so that no C++ object is exposed to ctypes.
//
//   n = state dimension, m = measurement dimension.
//
// Inputs x and P are taken as the prior belief; the posterior belief is written
// back into x_out (n x 1) and P_out (n x n).
NUMERIX_EXPORT NUMERIXStatus NUMERIX_KalmanStep(
    const double* F, ///< [in]  state-transition matrix (n x n, row-major)
    const double* H, ///< [in]  measurement matrix (m x n, row-major)
    const double* Q, ///< [in]  process-noise covariance (n x n, row-major)
    const double* R, ///< [in]  measurement-noise covariance (m x m, row-major)
    const double* x, ///< [in]  prior state estimate (n x 1)
    const double* P, ///< [in]  prior estimate covariance (n x n, row-major)
    const double* z, ///< [in]  measurement vector (m x 1)
    const NUMERIXInt n, ///< [in]  state dimension
    const NUMERIXInt m, ///< [in]  measurement dimension
    double* x_out,   ///< [out] posterior state estimate (n x 1)
    double* P_out    ///< [out] posterior estimate covariance (n x n, row-major)
)
{
    NUMERIX_TRY
    numerix::KalmanFilter kf(to_matrix(F, n, n),
                          to_matrix(H, m, n),
                          to_matrix(Q, n, n),
                          to_matrix(R, m, m),
                          to_matrix(x, n, 1),
                          to_matrix(P, n, n));

    kf.predict();
    kf.update(to_matrix(z, m, 1));

    from_matrix(kf.state(), x_out);
    from_matrix(kf.covariance(), P_out);
    return NUMERIXStatus::Success;
    NUMERIX_CATCH
    return NUMERIXStatus::Failed;
}
