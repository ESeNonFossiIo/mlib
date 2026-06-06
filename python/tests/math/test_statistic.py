import math
import pytest

from numerixpy.math.statistic import mean, moment, stddev, var


_DATA = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0]
_MEAN = 5.0
_VAR = 4.0
_STD = 2.0


def test_mean():
    assert math.isclose(mean(_DATA), _MEAN)


def test_var():
    assert math.isclose(var(_DATA), _VAR)


def test_stddev():
    assert math.isclose(stddev(_DATA), _STD)


def test_mean_single_element():
    assert math.isclose(mean([42.0]), 42.0)


def test_var_single_element():
    assert math.isclose(var([42.0]), 0.0)


def test_stddev_single_element():
    assert math.isclose(stddev([42.0]), 0.0)


def test_moment_order1_equals_mean():
    assert math.isclose(moment(_DATA, order=1), mean(_DATA))


def test_moment_central_order2_equals_var():
    assert math.isclose(moment(_DATA, order=2, central=True), var(_DATA))


def test_mean_uniform():
    v = [1.0, 2.0, 3.0, 4.0, 5.0]
    assert math.isclose(mean(v), 3.0)
