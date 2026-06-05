#ifndef MLIB_MATH_KALMAN_FILTER_H
#define MLIB_MATH_KALMAN_FILTER_H

#include "mlib/math/matrix/matrix.h"

/** \addtogroup math
 *  @{
 */
namespace mlib {

// ----------------------------------------------------------------------------
// \brief Discrete-time linear Kalman filter.
//
// The Kalman filter is a recursive estimator: it tracks the hidden state of a
// linear dynamical system that is observed through noisy measurements, and at
// every step produces the state estimate that minimises the mean squared
// error. It keeps two quantities:
//
//   - x : the current best estimate of the state            (n x 1 column)
//   - P : the covariance of that estimate (our uncertainty) (n x n)
//
// and assumes the system obeys the linear-Gaussian model
//
//   x_k = F x_{k-1} + B u_k + w,   w ~ N(0, Q)   (process / motion model)
//   z_k = H x_k             + v,   v ~ N(0, R)   (measurement model)
//
// where
//   F : state-transition matrix  (n x n) — how the state evolves on its own
//   B : control matrix           (n x l) — how a known input u_k pushes it
//   H : measurement matrix       (m x n) — how the state maps to observations
//   Q : process-noise covariance (n x n) — trust in the motion model
//   R : measurement-noise cov.   (m x m) — trust in the sensor
//
// Each cycle has two phases. **Predict** advances the state with the motion
// model and *grows* the covariance (uncertainty increases when we extrapolate):
//
//   x  <-  F x  (+ B u)
//   P  <-  F P Fᵀ + Q
//
// **Update** folds in a new measurement z and *shrinks* the covariance. The
// innovation y is the surprise (measured minus predicted), S is its
// covariance, and the Kalman gain K decides how much of the surprise to trust:
//
//   y  =  z - H x                 (innovation / residual)
//   S  =  H P Hᵀ + R              (innovation covariance)
//   K  =  P Hᵀ S⁻¹                (Kalman gain)
//   x  <-  x + K y
//   P  <-  (I - K H) P
//
// Intuitively K balances the two noise sources: when R is large (a noisy
// sensor) K is small and the filter leans on its prediction; when P is large
// (an uncertain prediction) K is large and the filter follows the measurement.
//
// \note All matrices use mlib::Matrixd. State and measurement vectors are
//       passed as single-column matrices (n x 1 and m x 1 respectively).
//
// \code
//   // Constant-velocity 1-D tracker: state = [position, velocity].
//   const double dt = 1.0;
//   mlib::Matrixd F{{1.0, dt}, {0.0, 1.0}};   // x += v·dt
//   mlib::Matrixd H{{1.0, 0.0}};              // we only measure position
//   mlib::Matrixd Q{{1e-3, 0.0}, {0.0, 1e-3}};
//   mlib::Matrixd R{{0.1}};
//   mlib::Matrixd x0{{0.0}, {0.0}};
//   mlib::Matrixd P0{{1.0, 0.0}, {0.0, 1.0}};
//
//   mlib::KalmanFilter kf(F, H, Q, R, x0, P0);
//   kf.predict();
//   kf.update(mlib::Matrixd{{1.2}});          // a position reading of 1.2
//   const mlib::Matrixd& estimate = kf.state();
// \endcode
class KalmanFilter {
public:
    // ------------------------------------------------------------------------
    // \brief Build a filter from the model matrices and the initial belief.
    //
    // \note The dimensions must be mutually consistent: with an n-dimensional
    //       state and m-dimensional measurement, F/Q/P0 are n x n, H is m x n,
    //       R is m x m and x0 is n x 1.
    KalmanFilter(
        const Matrixd& F,  ///< [in] state-transition matrix (n x n)
        const Matrixd& H,  ///< [in] measurement matrix (m x n)
        const Matrixd& Q,  ///< [in] process-noise covariance (n x n)
        const Matrixd& R,  ///< [in] measurement-noise covariance (m x m)
        const Matrixd& x0, ///< [in] initial state estimate (n x 1)
        const Matrixd& P0  ///< [in] initial estimate covariance (n x n)
    );

    // ------------------------------------------------------------------------
    // \brief Time-update with no control input.
    //
    // Propagates the state through the motion model and inflates the
    // covariance by the process noise:
    //   x <- F x ,   P <- F P Fᵀ + Q.
    void predict();

    // ------------------------------------------------------------------------
    // \brief Time-update driven by a known control input.
    //
    // Same as predict() but adds the deterministic effect of the control:
    //   x <- F x + B u ,   P <- F P Fᵀ + Q.
    void predict(
        const Matrixd& B, ///< [in] control matrix (n x l)
        const Matrixd& u  ///< [in] control vector (l x 1)
    );

    // ------------------------------------------------------------------------
    // \brief Measurement-update: correct the state with an observation z.
    //
    // Computes the Kalman gain and blends prediction and measurement:
    //   y = z - H x , S = H P Hᵀ + R , K = P Hᵀ S⁻¹ ,
    //   x <- x + K y , P <- (I - K H) P.
    void update(const Matrixd& z ///< [in] measurement vector (m x 1)
    );

    // ------------------------------------------------------------------------
    // \brief Current state estimate (n x 1).
    // \return reference to the internal state vector x.
    const Matrixd& state() const;

    // ------------------------------------------------------------------------
    // \brief Current estimate covariance (n x n).
    // \return reference to the internal covariance matrix P.
    const Matrixd& covariance() const;

    // ------------------------------------------------------------------------
    // \brief Innovation y = z - H x for a candidate measurement.
    //
    // The residual between the measurement and what the current state predicts;
    // useful for gating / outlier rejection before calling update().
    // \return the (m x 1) innovation vector.
    Matrixd innovation(const Matrixd& z ///< [in] measurement vector (m x 1)
    ) const;

private:
    Matrixd F; ///< state-transition matrix (n x n)
    Matrixd H; ///< measurement matrix (m x n)
    Matrixd Q; ///< process-noise covariance (n x n)
    Matrixd R; ///< measurement-noise covariance (m x m)
    Matrixd x; ///< state estimate (n x 1)
    Matrixd P; ///< estimate covariance (n x n)
};

} // namespace mlib
/** @}*/
#endif // MLIB_MATH_KALMAN_FILTER_H
