import math
import pytest

from numerixpy.math.math_utility import truncate_decimals


def test_truncate_one_decimal():
    # size=0.1 → rounds to 1 decimal place (int(num/0.1)*0.1)
    result = truncate_decimals(3.14159, 0.1)
    assert math.isclose(result, 3.1, abs_tol=1e-9)


def test_truncate_two_decimals():
    result = truncate_decimals(3.14159, 0.01)
    assert math.isclose(result, 3.14, abs_tol=1e-9)


def test_truncate_integer():
    result = truncate_decimals(3.7, 1.0)
    assert math.isclose(result, 3.0, abs_tol=1e-9)


def test_truncate_negative():
    result = truncate_decimals(-2.75, 0.1)
    assert math.isclose(result, -2.7, abs_tol=1e-9)


def test_truncate_zero():
    result = truncate_decimals(0.0, 0.1)
    assert math.isclose(result, 0.0)
