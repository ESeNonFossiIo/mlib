"""Discrete-time linear Kalman filter (Python wrapper over the C++ binding).

The heavy lifting (matrix algebra and the predict/update equations) lives in
the compiled ``numerix::KalmanFilter`` class; this module marshals the model and
the running belief across the ctypes boundary as flat, row-major buffers.

See :class:`KalmanFilter` for the model definition. A single call to
:meth:`KalmanFilter.step` performs one *predict + update* cycle and advances
the stored state ``x`` and covariance ``P`` in place.
"""

from ctypes import POINTER, c_double, c_uint64
from typing import List, Sequence

from numerixpy.bind.load_symbols import evaluateFunction

Matrix = Sequence[Sequence[float]]
Vector = Sequence[float]


def _flatten(m: Matrix) -> List[float]:
    """Flatten a row-major 2-D sequence into a 1-D list of floats."""
    return [float(value) for row in m for value in row]


def _as_buffer(values: Sequence[float]):
    """Pack a flat sequence of floats into a ctypes double array."""
    return (c_double * len(values))(*values)


class KalmanFilter:
    """Recursive estimator for a linear-Gaussian state-space model.

    The filter tracks a hidden state observed through noisy measurements and
    returns, at every step, the minimum-mean-squared-error estimate. It assumes

        x_k = F x_{k-1} + w,  w ~ N(0, Q)   (motion model)
        z_k = H x_k     + v,  v ~ N(0, R)   (measurement model)

    and maintains the state estimate ``x`` (n-vector) together with its
    covariance ``P`` (n x n). Each :meth:`step` runs one *predict* phase (push
    the state forward with ``F`` and grow ``P`` by ``Q``) followed by one
    *update* phase (fold in a measurement and shrink ``P``).

    Args:
        F: State-transition matrix, ``n x n``.
        H: Measurement matrix, ``m x n``.
        Q: Process-noise covariance, ``n x n``.
        R: Measurement-noise covariance, ``m x m``.
        x0: Initial state estimate, length ``n``.
        P0: Initial estimate covariance, ``n x n``.

    Example:
        >>> kf = KalmanFilter(
        ...     F=[[1.0, 1.0], [0.0, 1.0]],   # constant-velocity model
        ...     H=[[1.0, 0.0]],               # observe position only
        ...     Q=[[1e-3, 0.0], [0.0, 1e-3]],
        ...     R=[[0.25]],
        ...     x0=[0.0, 0.0],
        ...     P0=[[1.0, 0.0], [0.0, 1.0]],
        ... )
        >>> kf.step([1.1])     # one predict + update cycle
        >>> kf.state[0]        # filtered position
    """

    def __init__(
        self,
        F: Matrix,
        H: Matrix,
        Q: Matrix,
        R: Matrix,
        x0: Vector,
        P0: Matrix,
    ) -> None:
        self._n = len(x0)
        self._m = len(H)
        if any(len(row) != self._n for row in F) or len(F) != self._n:
            raise ValueError("F must be square with side len(x0)")
        if any(len(row) != self._n for row in H):
            raise ValueError("H must have len(x0) columns")
        self._F = _flatten(F)
        self._H = _flatten(H)
        self._Q = _flatten(Q)
        self._R = _flatten(R)
        #: Current state estimate as a flat list of length ``n``.
        self.state: List[float] = [float(v) for v in x0]
        #: Current estimate covariance as a flat, row-major list (``n x n``).
        self.covariance: List[float] = _flatten(P0)

    def step(self, z: Vector) -> List[float]:
        """Run one predict + update cycle for the measurement ``z``.

        Advances :attr:`state` and :attr:`covariance` in place and returns the
        updated state for convenience.

        Args:
            z: Measurement vector of length ``m``.

        Returns:
            The posterior state estimate (also stored in :attr:`state`).

        Raises:
            ValueError: If ``z`` does not have length ``m``.
            RuntimeError: If the C++ binding reports a failure.
        """
        if len(z) != self._m:
            raise ValueError("z must have length %d" % self._m)

        x_out = (c_double * self._n)()
        p_out = (c_double * (self._n * self._n))()

        status = evaluateFunction(
            "KalmanStep",
            [
                POINTER(c_double),  # F
                POINTER(c_double),  # H
                POINTER(c_double),  # Q
                POINTER(c_double),  # R
                POINTER(c_double),  # x
                POINTER(c_double),  # P
                POINTER(c_double),  # z
                c_uint64,           # n
                c_uint64,           # m
                POINTER(c_double),  # x_out
                POINTER(c_double),  # P_out
            ],
            [
                _as_buffer(self._F),
                _as_buffer(self._H),
                _as_buffer(self._Q),
                _as_buffer(self._R),
                _as_buffer(self.state),
                _as_buffer(self.covariance),
                _as_buffer([float(v) for v in z]),
                c_uint64(self._n),
                c_uint64(self._m),
                x_out,
                p_out,
            ],
            c_uint64,
        )

        if status != 1:  # NUMERIXStatus::Success == 1
            raise RuntimeError("KalmanStep failed (status=%d)" % status)

        self.state = list(x_out)
        self.covariance = list(p_out)
        return self.state
