#include "../test.h"

#include "numerix/math/kalman_filter.h"

using namespace numerix;

// 1-D constant-velocity tracker.
//   state  = [position, velocity]ᵀ
//   sensor = position only
//
// The test drives the full filter cycle (predict / predict-with-control /
// update / innovation) so every public method and code path is exercised.
int main()
{
    print_title("KalmanFilter");

    const double dt = 1.0;

    // x_{k} = F x_{k-1} :  pos += vel·dt, vel unchanged.
    Matrixd F{{1.0, dt}, {0.0, 1.0}};
    // We only observe the position.
    Matrixd H{{1.0, 0.0}};
    // Small process noise (we trust the constant-velocity model).
    Matrixd Q{{1e-3, 0.0}, {0.0, 1e-3}};
    // Measurement noise variance of the position sensor.
    Matrixd R{{0.25}};
    // Initial belief: at the origin, at rest, but quite uncertain.
    Matrixd x0{{0.0}, {0.0}};
    Matrixd P0{{1.0, 0.0}, {0.0, 1.0}};

    KalmanFilter kf(F, H, Q, R, x0, P0);

    std::cout << "Initial state" << kf.state();
    std::cout << "Initial covariance" << kf.covariance();

    // A target moving at ~1 unit/step, measured with some noise.
    const double measurements[] = {1.1, 1.9, 3.2, 3.9, 5.1};

    for (std::size_t k = 0; k < 5; ++k) {
        kf.predict();

        Matrixd z{{measurements[k]}};
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "step " << k << " : measurement = " << measurements[k] << std::endl;
        std::cout << "innovation (before update)" << kf.innovation(z);

        kf.update(z);

        std::cout << "state" << kf.state();
        std::cout << "covariance" << kf.covariance();
    }

    // Exercise the control-input predict overload: a known acceleration kick.
    std::cout << "========================================" << std::endl;
    std::cout << "predict with control input" << std::endl;
    Matrixd B{{0.5 * dt * dt}, {dt}}; // constant-acceleration control matrix
    Matrixd u{{0.2}};                 // applied acceleration
    kf.predict(B, u);
    std::cout << "state" << kf.state();
    std::cout << "covariance" << kf.covariance();

    return 0;
}
