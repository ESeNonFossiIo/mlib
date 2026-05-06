import math
import pytest

from mlibpy.math.statistics.norm_dist import norm_cdf, norm_pdf

_INV_SQRT_TWO_PI = 1.0 / math.sqrt(2.0 * math.pi)


def test_pdf_at_zero():
    assert math.isclose(norm_pdf(0.0), _INV_SQRT_TWO_PI, rel_tol=1e-7)


def test_pdf_positive():
    assert math.isclose(norm_pdf(1.0), _INV_SQRT_TWO_PI * math.exp(-0.5), rel_tol=1e-7)


def test_pdf_symmetric():
    assert math.isclose(norm_pdf(-1.0), norm_pdf(1.0), rel_tol=1e-10)


def test_cdf_at_zero():
    assert math.isclose(norm_cdf(0.0), 0.5, abs_tol=1e-7)


def test_cdf_positive_large():
    assert norm_cdf(5.0) > 0.999


def test_cdf_negative_large():
    assert norm_cdf(-5.0) < 0.001


def test_cdf_symmetry():
    for x in [0.5, 1.0, 1.5, 2.0]:
        assert math.isclose(norm_cdf(x) + norm_cdf(-x), 1.0, abs_tol=1e-7)


def test_cdf_known_value_1():
    assert math.isclose(norm_cdf(1.0), 0.8413, abs_tol=1e-4)


def test_cdf_known_value_neg1():
    assert math.isclose(norm_cdf(-1.0), 0.1587, abs_tol=1e-4)
