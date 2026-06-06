#include <numerix/core/export.h>
#include <numerix/geo/coordinates.h>

#include "_python/status.h"

#include <cstdint>

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_UTMCentralMeridian(const int64_t zone, ///< [in]  UTM zone [1, 60]
                                               double* result ///< [out] central meridian in radians
)
{
    *result = numerix::UTM_central_meridian(static_cast<int>(zone));
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_ComputeZone(const double lon, ///< [in]  longitude in radians
                                        int64_t* result   ///< [out] UTM zone
)
{
    *result = static_cast<int64_t>(numerix::compute_zone(lon));
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_ComputeConvergenceAngle(
    const double lat, ///< [in]  latitude in radians
    const double lon, ///< [in]  longitude in radians
    double* result    ///< [out] convergence angle in degrees
)
{
    *result = numerix::compute_convergence_angle(lat, lon);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_ArclenOfAMeridian(const double lat, ///< [in]  latitude in radians
                                              double* result    ///< [out] arc length in metres
)
{
    *result = numerix::arclen_of_a_meridian(lat);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_FootpointLat(const double y, ///< [in]  UTM northing in metres
                                         double* result  ///< [out] footpoint latitude in radians
)
{
    *result = numerix::footpoint_lat(y);
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_UTMLatLonToXY(const double lat, ///< [in]  latitude in radians
                                          const double lon, ///< [in]  longitude in radians
                                          double* x,        ///< [out] UTM easting in metres
                                          double* y         ///< [out] UTM northing in metres
)
{
    const numerix::Point p = numerix::UTM_latlon_to_xy(lat, lon);
    *x = p.x();
    *y = p.y();
    return NUMERIXStatus::Success;
}

// ----------------------------------------------------------------------------
NUMERIX_EXPORT NUMERIXStatus NUMERIX_UTMXYToLatLon(
    const double x,           ///< [in]  UTM easting in metres
    const double y,           ///< [in]  UTM northing in metres
    const int64_t zone,       ///< [in]  UTM zone
    const NUMERIXInt south_hemi, ///< [in]  1 = southern hemisphere
    double* lat,              ///< [out] latitude in radians
    double* lon               ///< [out] longitude in radians
)
{
    const numerix::Point p(x, y);
    numerix::UTM_xy_to_latlon(p, static_cast<int>(zone), south_hemi != 0, *lat, *lon);
    return NUMERIXStatus::Success;
}
