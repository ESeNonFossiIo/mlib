#include "mlib/math/kalman_filter.h"

namespace mlib {

// ----------------------------------------------------------------------------
KalmanFilter::KalmanFilter(
    const Matrixd& F,
    const Matrixd& H,
    const Matrixd& Q,
    const Matrixd& R,
    const Matrixd& x0,
    const Matrixd& P0
)
    : F(F), H(H), Q(Q), R(R), x(x0), P(P0)
{
}

// ----------------------------------------------------------------------------
void KalmanFilter::predict()
{
    // Project the state and covariance forward through the motion model.
    // The Fᵀ on both sides of P rotates the uncertainty into the new state
    // basis; adding Q grows it to account for unmodelled disturbances.
    x = F * x;
    P = F * P * F.t() + Q;
}

// ----------------------------------------------------------------------------
void KalmanFilter::predict(const Matrixd& B, const Matrixd& u)
{
    // Same as the uncontrolled predict, plus the deterministic shift B u that
    // a known input applies to the state.
    x = F * x + B * u;
    P = F * P * F.t() + Q;
}

// ----------------------------------------------------------------------------
void KalmanFilter::update(const Matrixd& z)
{
    // Innovation: how far the measurement is from what we predicted.
    const Matrixd y = z - H * x;
    // Innovation covariance: predicted measurement uncertainty plus sensor noise.
    const Matrixd S = H * P * H.t() + R;
    // Kalman gain: maps the innovation (measurement space) back into a state
    // correction, weighted by how much we trust the measurement vs. the model.
    const Matrixd K = P * H.t() * S.inv();

    x = x + K * y;

    // Joseph-free covariance update: shrink P by the information just gained.
    const IdentityMatrix I(P.r());
    P = (I - K * H) * P;
}

// ----------------------------------------------------------------------------
Matrixd KalmanFilter::innovation(const Matrixd& z) const
{
    return z - H * x;
}

// ----------------------------------------------------------------------------
const Matrixd& KalmanFilter::state() const
{
    return x;
}

// ----------------------------------------------------------------------------
const Matrixd& KalmanFilter::covariance() const
{
    return P;
}

} // namespace mlib
