import math
import pytest

from numerixpy.geo.coordinates import (
    arclen_of_a_meridian,
    compute_convergence_angle,
    compute_zone,
    footpoint_lat,
    utm_central_meridian,
    utm_latlon_to_xy,
    utm_xy_to_latlon,
)

# All geo C++ functions work in degrees; arclen_of_a_meridian works in radians.
_DEG = math.pi / 180.0


def test_utm_central_meridian_zone_32():
    # zone 32 → central meridian 9° E (returned in radians)
    cm = utm_central_meridian(32)
    assert math.isclose(cm, 9.0 * _DEG, rel_tol=1e-6)


def test_utm_central_meridian_zone_1():
    cm = utm_central_meridian(1)
    assert math.isclose(cm, -177.0 * _DEG, rel_tol=1e-6)


def test_compute_zone_germany():
    # 9° E is in UTM zone 32
    assert compute_zone(9.0) == 32


def test_compute_zone_new_york():
    # −74° is in UTM zone 18
    assert compute_zone(-74.0) == 18


def test_arclen_equator():
    assert math.isclose(arclen_of_a_meridian(0.0), 0.0, abs_tol=1e-3)


def test_arclen_positive():
    assert arclen_of_a_meridian(45.0 * _DEG) > 0.0


def test_footpoint_lat_equator():
    result = footpoint_lat(0.0)
    assert math.isclose(result, 0.0, abs_tol=1e-6)


def test_utm_roundtrip():
    # Paris in degrees; UTM_xy_to_latlon also returns degrees
    lat = 48.8566
    lon = 2.3522
    x, y = utm_latlon_to_xy(lat, lon)
    lat2, lon2 = utm_xy_to_latlon(x, y, zone=31)
    assert math.isclose(lat2, lat, abs_tol=1e-3)
    assert math.isclose(lon2, lon, abs_tol=1e-3)


def test_convergence_angle_on_meridian():
    # At exactly the central meridian of zone 32 (lon = 9°) the angle is 0
    angle = compute_convergence_angle(45.0, 9.0)
    assert math.isclose(angle, 0.0, abs_tol=1e-9)
