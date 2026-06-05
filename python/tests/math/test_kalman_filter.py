"""Tests for the Kalman filter Python binding.

The reference model throughout is a 1-D constant-velocity tracker:

    state  = [position, velocity]
    sensor = position only, with measurement-noise variance R.

A ground-truth target starts at the origin and moves at +1 unit/step; the
filter is fed noisy position readings and is expected to recover both the
position and the (unobserved) velocity.
"""

import pytest

from mlibpy.math.kalman_filter import KalmanFilter

# ---------------------------------------------------------------------------
# Reference constant-velocity model
# ---------------------------------------------------------------------------
_F = [[1.0, 1.0], [0.0, 1.0]]
_H = [[1.0, 0.0]]
_Q = [[1e-3, 0.0], [0.0, 1e-3]]
_R = [[0.25]]
_X0 = [0.0, 0.0]
_P0 = [[1.0, 0.0], [0.0, 1.0]]


def _make_filter() -> KalmanFilter:
    return KalmanFilter(F=_F, H=_H, Q=_Q, R=_R, x0=list(_X0), P0=_P0)


# ---------------------------------------------------------------------------
# Single-step behaviour
# ---------------------------------------------------------------------------


def test_step_returns_state():
    """step() returns the same list stored in .state."""
    kf = _make_filter()
    returned = kf.step([1.0])
    assert returned == kf.state
    assert len(kf.state) == 2


def test_first_step_pulls_toward_measurement():
    """The first position estimate lies between the prior (0) and the reading."""
    kf = _make_filter()
    kf.step([1.0])
    assert 0.0 < kf.state[0] < 1.0


def test_covariance_shrinks_after_update():
    """A measurement reduces the position-variance entry P[0, 0]."""
    kf = _make_filter()
    prior_var = kf.covariance[0]  # P0[0,0] == 1.0
    kf.step([1.0])
    assert kf.covariance[0] < prior_var


# ---------------------------------------------------------------------------
# Convergence over a trajectory
# ---------------------------------------------------------------------------


def test_tracks_constant_velocity_target():
    """After several clean readings the filter recovers position and velocity."""
    kf = _make_filter()
    measurements = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    for z in measurements:
        kf.step([z])
    # Position should be close to the last reading...
    assert abs(kf.state[0] - 8.0) < 0.5
    # ...and the estimated velocity should converge near the true +1/step.
    assert abs(kf.state[1] - 1.0) < 0.3


def test_estimate_stays_finite():
    """The covariance never blows up or goes NaN over a long run."""
    kf = _make_filter()
    for k in range(50):
        kf.step([float(k)])
    assert all(v == v for v in kf.covariance)  # no NaN
    assert kf.covariance[0] < 1.0


# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------


def test_rejects_wrong_measurement_length():
    """A measurement of the wrong dimension is rejected before the C call."""
    kf = _make_filter()
    with pytest.raises(ValueError):
        kf.step([1.0, 2.0])


def test_rejects_non_square_transition():
    """A non-square F is caught at construction time."""
    with pytest.raises(ValueError):
        KalmanFilter(
            F=[[1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            H=_H,
            Q=_Q,
            R=_R,
            x0=_X0,
            P0=_P0,
        )


def test_rejects_mismatched_measurement_matrix():
    """H with the wrong number of columns is rejected at construction time."""
    with pytest.raises(ValueError):
        KalmanFilter(
            F=_F,
            H=[[1.0, 0.0, 0.0]],
            Q=_Q,
            R=_R,
            x0=_X0,
            P0=_P0,
        )
