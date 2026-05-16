#include "../test.h"

#include "mlib/geo/coordinates.h"

#include <iomanip>
#include <iostream>

using namespace mlib;

// Coverage entry point for the geo/coordinates module.  Exercises the
// helper conversions (zone, central meridian, footpoint latitude,
// transverse-Mercator arc length) and the round-trip UTM <-> lat/lon paths
// in both hemispheres.
int main()
{
    print_title("Coordinates 01");

    std::cout << std::fixed << std::setprecision(6);

    // ------------------------------------------------------------------
    // Static helpers
    // ------------------------------------------------------------------
    std::cout << "zone(0)       = " << compute_zone(0.0) << std::endl;
    std::cout << "zone(12.15)   = " << compute_zone(12.153571) << std::endl;
    std::cout << "zone(-122)    = " << compute_zone(-122.0) << std::endl;
    std::cout << "zone(190)     = " << compute_zone(190.0) << std::endl;

    std::cout << "cm(32) rad    = " << UTM_central_meridian(32) << std::endl;
    std::cout << "cm(1)  rad    = " << UTM_central_meridian(1) << std::endl;

    const double lat_rad = Angle(42.497812, AngleType::deg).rad();
    std::cout << "arclen        = " << arclen_of_a_meridian(lat_rad) << std::endl;
    std::cout << "footpoint     = " << footpoint_lat(4707540.0) << std::endl;

    // ------------------------------------------------------------------
    // Northern hemisphere round-trip
    // ------------------------------------------------------------------
    const double lat_n = 42.497812;
    const double lon_n = 12.153571;

    Point p_n  = UTM_latlon_to_xy(lat_n, lon_n);
    int zone_n = compute_zone(lon_n);
    double lat_back_n = 0.0;
    double lon_back_n = 0.0;
    UTM_xy_to_latlon(p_n, zone_n, /*southhemi=*/false, lat_back_n, lon_back_n);
    std::cout << "north xy      = (" << p_n[0] << ", " << p_n[1] << ")" << std::endl;
    std::cout << "north lat/lon = (" << lat_back_n << ", " << lon_back_n << ")" << std::endl;

    // ------------------------------------------------------------------
    // Southern hemisphere round-trip — exercises the y -= 1e7 adjustment
    // ------------------------------------------------------------------
    const double lat_s = -33.868820;
    const double lon_s = 151.209296; // Sydney

    Point p_s  = UTM_latlon_to_xy(lat_s, lon_s);
    int zone_s = compute_zone(lon_s);
    double lat_back_s = 0.0;
    double lon_back_s = 0.0;
    UTM_xy_to_latlon(p_s, zone_s, /*southhemi=*/true, lat_back_s, lon_back_s);
    std::cout << "south xy      = (" << p_s[0] << ", " << p_s[1] << ")" << std::endl;
    std::cout << "south lat/lon = (" << lat_back_s << ", " << lon_back_s << ")" << std::endl;

    // ------------------------------------------------------------------
    // Direct latlon_to_xy / xy_to_latlon (without the UTM scale)
    // ------------------------------------------------------------------
    Point p_tm({0.0, 0.0});
    const double lon0 = UTM_central_meridian(zone_n);
    latlon_to_xy(Angle(lat_n, AngleType::deg).rad(),
                 Angle(lon_n, AngleType::deg).rad(),
                 lon0,
                 p_tm);
    double lat_tm = 0.0;
    double lon_tm = 0.0;
    xy_to_latlon(p_tm, lon0, lat_tm, lon_tm);
    std::cout << "tm xy         = (" << p_tm[0] << ", " << p_tm[1] << ")" << std::endl;
    std::cout << "tm lat/lon    = (" << lat_tm << ", " << lon_tm << ")" << std::endl;

    // ------------------------------------------------------------------
    // Convergence angle
    // ------------------------------------------------------------------
    std::cout << "conv(42,12)   = " << compute_convergence_angle(lat_n, lon_n) << std::endl;
    std::cout << "conv(-33,151) = " << compute_convergence_angle(lat_s, lon_s) << std::endl;

    return 0;
}
