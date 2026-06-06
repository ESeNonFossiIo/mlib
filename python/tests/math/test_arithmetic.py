import math
import pytest

from numerixpy.math.arithmetic import argmax, argmin, max, min, normalize, pythagoras


def test_pythagoras_3_4():
    assert math.isclose(pythagoras(3.0, 4.0), 5.0)


def test_pythagoras_symmetric():
    assert math.isclose(pythagoras(5.0, 12.0), pythagoras(12.0, 5.0))


def test_pythagoras_zero():
    assert math.isclose(pythagoras(0.0, 7.0), 7.0)


def test_argmax_basic():
    assert argmax([1.0, 3.0, 2.0]) == 1


def test_argmax_first():
    assert argmax([9.0, 1.0, 2.0]) == 0


def test_argmin_basic():
    assert argmin([3.0, 1.0, 2.0]) == 1


def test_argmin_last():
    assert argmin([3.0, 2.0, 0.5]) == 2


def test_max_basic():
    assert math.isclose(max([1.0, 5.0, 3.0]), 5.0)


def test_min_basic():
    assert math.isclose(min([4.0, 2.0, 7.0]), 2.0)


def test_normalize_range():
    result = normalize([0.0, 5.0, 10.0])
    assert math.isclose(result[0], 0.0)
    assert math.isclose(result[-1], 1.0)
    assert all(0.0 <= v <= 1.0 for v in result)


def test_normalize_constant_vector():
    # All equal → max==min → 0/0 → NaN; this is the defined behaviour of numerix::normalize
    result = normalize([3.0, 3.0, 3.0])
    assert all(math.isnan(v) for v in result)
