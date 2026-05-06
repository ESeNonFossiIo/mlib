#include <mlib/core/export.h>
#include <mlib/geo/coordinates.h>

#include "_python/status.h"

#include <cstdint>

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_UTMCentralMeridian(
    const int64_t zone,   ///< [in]  UTM zone [1, 60]
    double*       result  ///< [out] central meridian in radians
)
{
    *result = mlib::UTM_central_meridian(static_cast<int>(zone));
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_ComputeZone(
    const double lon,   ///< [in]  longitude in radians
    int64_t*     result ///< [out] UTM zone
)
{
    *result = static_cast<int64_t>(mlib::compute_zone(lon));
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_ComputeConvergenceAngle(
    const double lat,   ///< [in]  latitude in radians
    const double lon,   ///< [in]  longitude in radians
    double*      result ///< [out] convergence angle in degrees
)
{
    *result = mlib::compute_convergence_angle(lat, lon);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_ArclenOfAMeridian(
    const double lat,   ///< [in]  latitude in radians
    double*      result ///< [out] arc length in metres
)
{
    *result = mlib::arclen_of_a_meridian(lat);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_FootpointLat(
    const double y,     ///< [in]  UTM northing in metres
    double*      result ///< [out] footpoint latitude in radians
)
{
    *result = mlib::footpoint_lat(y);
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_UTMLatLonToXY(
    const double lat,  ///< [in]  latitude in radians
    const double lon,  ///< [in]  longitude in radians
    double*      x,    ///< [out] UTM easting in metres
    double*      y     ///< [out] UTM northing in metres
)
{
    const mlib::Point p = mlib::UTM_latlon_to_xy(lat, lon);
    *x = p.x();
    *y = p.y();
    return MLIBStatus::Success;
}

// ----------------------------------------------------------------------------
MLIB_EXPORT MLIBStatus MLIB_UTMXYToLatLon(
    const double  x,         ///< [in]  UTM easting in metres
    const double  y,         ///< [in]  UTM northing in metres
    const int64_t zone,      ///< [in]  UTM zone
    const MLIBInt south_hemi,///< [in]  1 = southern hemisphere
    double*       lat,       ///< [out] latitude in radians
    double*       lon        ///< [out] longitude in radians
)
{
    const mlib::Point p(x, y);
    mlib::UTM_xy_to_latlon(p, static_cast<int>(zone), south_hemi != 0, *lat, *lon);
    return MLIBStatus::Success;
}
