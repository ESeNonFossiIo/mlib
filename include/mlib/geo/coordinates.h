#ifndef  __m_GEO_COORDINATES_H__
#define  __m_GEO_COORDINATES_H__

#include <cmath>

#include "mlib/math/constants.h"

#include "mlib/math/angle.h"
#include "mlib/math/point.h"

/** \addtogroup geo
 *  @{
 */
namespace _mlib
{
  struct ModelConstants
  {
    double sm_a;
    double sm_b;
    double sm_EccSquared;
    double UTMScaleFactor;
  };

  ModelConstants EMC =
  {
    /* .sm_a = */ 6378137.0,
    /* .sm_b = */ 6356752.314,
    /* .sm_EccSquared = */ 6.69437999013e-03,
    /* .UTMScaleFactor = */ 0.9996
  };

  /**
   * Computes the ellipsoidal distance from the equator to a point at a given latitude.
   * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
   * GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
   * @method arclen_of_a_meridian
   * @param  lat                  [description]
   * @param  model                [description]
   * @return                      [description]
   */
  double arclen_of_a_meridian(const double& lat,
                              const ModelConstants& model = EMC);

  /**
   * Determines the central meridian for the given UTM zone.
   * @method UTMCentralMeridian
   * @param  zone  An integer value designating the UTM zone, range [1,60].
   * @return       The central meridian for the given UTM zone, in radians, or zero
   *   if the UTM zone parameter is outside the range [1,60].
   *   Range of the central meridian is the radian equivalent of [-177,+177].
   */
  double UTM_central_meridian(const int& zone);

  /**
   * Computes the footpoint latitude for use in converting transverse Mercator
   * coordinates to ellipsoidal coordinates.
   *
   * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
   *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
   *
   * @method footpoint_lat
   * @param  y  The UTM northing coordinate, in meters.
   * @return    The footpoint latitude, in radians.
   */
  double footpoint_lat(const double& y,
                       const ModelConstants& model = EMC);

  /**
   * Converts a latitude/longitude pair to x and y coordinates in the
   * Transverse Mercator projection.  Note that Transverse Mercator is not
   * the same as UTM; a scale factor is required to convert between them.
   *
   * @method latlon_to_xy
   * @param  lat          Latitude of the point, in radians.
   * @param  lon          Longitude of the point, in radians.
   * @param  lon0         Longitude of the central meridian to be used, in radians.
   * @param  p            [description]
   * @param  model        [description]
   */
  void latlon_to_xy(const double& lat, const double& lon, const double& lon0,
                    Point& p,
                    const ModelConstants& model = EMC);

  /**
   * Converts x and y coordinates in the Transverse Mercator projection to
   * a latitude/longitude pair.  Note that Transverse Mercator is not
   * the same as UTM; a scale factor is required to convert between them.
   * @method xy_to_latlon
   * @param  p   The easting of the point, in meters and the northing
   * of the point, in meters.
   * @param  lat          [description]
   * @param  lon          [description]
   * @param  lon0         [description]
   * @param  model        [description]
   */
  void xy_to_latlon(const Point& p, const double& lon0,
                    double& lat, double& lon,
                    const ModelConstants& model = EMC);

  /**
   * Converts a latitude/longitude pair to x and y coordinates in the
   * Universal Transverse Mercator projection.
   * @method UTM_latlon_to_xy
   * @param  zone             [description]
   * @param  lat   Latitude of the point, in radians.
   * @param  lon   Longitude of the point, in radians.
   * @param  p     UTM zone to be used for calculating values for x and y.
   *               If zone is less than 1 or greater than 60, the routine
   *               will determine the appropriate zone from the value of lon.
   */
  Point UTM_latlon_to_xy(const double& lat, const double& lon,
                         const ModelConstants& model = EMC);


  /**
   * Converts x and y coordinates in the Universal Transverse Mercator
   * projection to a latitude/longitude pair.
   * @method UTM_xy_to_latlon
   * @param  p                [description]
   * @param  zone       The UTM zone in which the point lies.
   * @param  lat              [description]
   * @param  lon              [description]
   * @param  southhemi  True if the point is in the southern hemisphere;
   *               false otherwise.
   * @param  model            [description]
   */
  void UTM_xy_to_latlon(const Point& p,
                        const int& zone, const bool& southhemi,
                        double& lat, double& lon,
                        const ModelConstants& model = EMC);

  /**
   * [compute_zone description]
   * @method compute_zone
   * @param  lon          [description]
   * @return              [description]
   */
  int compute_zone(const double& lon);

  /**
   * [compute_convergence_angle description]
   * @method compute_convergence_angle
   * @param  lat                       [description]
   * @param  lon                       [description]
   * @return convergence angle in deg.
   */
  double compute_convergence_angle(const double& lat,
                                   const double& lon);

}

/** @}*/
#endif // __m_GEO_COORDINATES_H__
