import math
import pytest

from mlibpy.utility.string_utils import (
    double_to_str,
    lower_case,
    str_to_bool,
    str_to_double,
    str_to_int,
    upper_case,
)


def test_lower_case():
    assert lower_case("Hello World") == "hello world"


def test_lower_case_already_lower():
    assert lower_case("abc") == "abc"


def test_upper_case():
    assert upper_case("Hello World") == "HELLO WORLD"


def test_upper_case_already_upper():
    assert upper_case("ABC") == "ABC"


def test_str_to_double():
    assert math.isclose(str_to_double("3.14"), 3.14, rel_tol=1e-6)


def test_str_to_double_negative():
    assert math.isclose(str_to_double("-2.5"), -2.5)


def test_double_to_str_roundtrip():
    v = 1.5
    s = double_to_str(v)
    assert math.isclose(float(s), v, rel_tol=1e-6)


def test_str_to_int():
    assert str_to_int("42") == 42


def test_str_to_int_negative():
    assert str_to_int("-7") == -7


def test_str_to_bool_true():
    assert str_to_bool("true") is True


def test_str_to_bool_false():
    assert str_to_bool("false") is False
