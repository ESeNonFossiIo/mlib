from ctypes import POINTER, byref, c_double, c_int64, c_uint64
from typing import Tuple

from numerixpy.bind.load_symbols import evaluateFunction


def utm_central_meridian(zone: int) -> float:
    """Return the central meridian (radians) for a UTM zone [1, 60]."""
    result = c_double(0.0)
    evaluateFunction(
        "UTMCentralMeridian",
        [c_int64, POINTER(c_double)],
        [c_int64(zone), byref(result)],
        c_uint64,
    )
    return result.value


def compute_zone(lon: float) -> int:
    """Return the UTM zone for a longitude given in radians."""
    result = c_int64(0)
    evaluateFunction(
        "ComputeZone",
        [c_double, POINTER(c_int64)],
        [lon, byref(result)],
        c_uint64,
    )
    return int(result.value)


def compute_convergence_angle(lat: float, lon: float) -> float:
    """Return the grid convergence angle in degrees."""
    result = c_double(0.0)
    evaluateFunction(
        "ComputeConvergenceAngle",
        [c_double, c_double, POINTER(c_double)],
        [lat, lon, byref(result)],
        c_uint64,
    )
    return result.value


def arclen_of_a_meridian(lat: float) -> float:
    """Return the ellipsoidal arc length from the equator to latitude (radians)."""
    result = c_double(0.0)
    evaluateFunction(
        "ArclenOfAMeridian",
        [c_double, POINTER(c_double)],
        [lat, byref(result)],
        c_uint64,
    )
    return result.value


def footpoint_lat(y: float) -> float:
    """Return the footpoint latitude (radians) for a UTM northing y (metres)."""
    result = c_double(0.0)
    evaluateFunction(
        "FootpointLat",
        [c_double, POINTER(c_double)],
        [y, byref(result)],
        c_uint64,
    )
    return result.value


def utm_latlon_to_xy(lat: float, lon: float) -> Tuple[float, float]:
    """Convert (lat, lon) in radians to UTM (easting, northing) in metres."""
    x = c_double(0.0)
    y = c_double(0.0)
    evaluateFunction(
        "UTMLatLonToXY",
        [c_double, c_double, POINTER(c_double), POINTER(c_double)],
        [lat, lon, byref(x), byref(y)],
        c_uint64,
    )
    return x.value, y.value


def utm_xy_to_latlon(
    x: float, y: float, zone: int, south_hemi: bool = False
) -> Tuple[float, float]:
    """Convert UTM (easting, northing, zone) to (lat, lon) in radians."""
    lat = c_double(0.0)
    lon = c_double(0.0)
    evaluateFunction(
        "UTMXYToLatLon",
        [c_double, c_double, c_int64, c_uint64, POINTER(c_double), POINTER(c_double)],
        [x, y, c_int64(zone), c_uint64(1 if south_hemi else 0), byref(lat), byref(lon)],
        c_uint64,
    )
    return lat.value, lon.value
