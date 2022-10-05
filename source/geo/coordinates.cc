#include "mlib/geo/coordinates.h"

namespace mlib
{
  double arclen_of_a_meridian(const double& lat,
                              const ModelConstants& model)
  {
    double sum  = model.sm_a + model.sm_b;
    double diff = model.sm_a - model.sm_b;

    double n  = diff / sum;
    double n2 = n*n;
    double n3 = n2*n;
    double n4 = n2*n2;
    double n5 = n3*n2;

    double a = (sum / 2.0) * (1.0 + (n2 / 4.0) + (n4 / 64.0));
    double b = (-3.0 * n / 2.0) + (9.0 * n3 / 16.0) + (-3.0 * n5 / 32.0);
    double c = (15.0 * n2 / 16.0) + (-15.0 * n4 / 32.0);
    double d = (-35.0 * n3 / 48.0) + (105.0 * n5 / 256.0);
    double e = (315.0 * n4 / 512.0);

    return a * (lat +
                (b * sin(2.0 * lat)) +
                (c * sin(4.0 * lat)) +
                (d * sin(6.0 * lat)) +
                (e * sin(8.0 * lat))
               );
  }

  double UTM_central_meridian(const int& zone)
  {
    return Angle(-183.0 + ((double)zone * 6.0), AngleType::deg).rad();
  }

  double footpoint_lat(const double& y,
                       const ModelConstants& model)
  {
    double sum  = model.sm_a + model.sm_b;
    double diff = model.sm_a - model.sm_b;

    double n  = diff / sum;
    double n2 = n*n;
    double n3 = n2*n;
    double n4 = n2*n2;
    double n5 = n3*n2;

    double a  = (sum / 2.0) * (1.0 + (n2 / 4.0) + (n4 / 64.0));
    double ya = y/a;

    double b  = (3.0 * n / 2.0) + (-27.0 * n3 / 32.0) + (269.0 * n5 / 512.0);
    double c  = (21.0 * n2 / 16.0) + (-55.0 * n4 / 32.0);
    double d  = (151.0 * n3 / 96.0) + (-417.0 * n5 / 128.0);
    double e  = (1097.0 * n4 / 512.0);

    return ya * (b * sin(2.0 * ya)) +
           (c * sin(4.0 * ya)) +
           (d * sin(6.0 * ya)) +
           (e * sin(8.0 * ya));
  }

  void latlon_to_xy(const double& lat, const double& lon, const double& lon0,
                    Point& p,
                    const ModelConstants& model)
  {
    double sm_a2 = model.sm_a * model.sm_a;
    double sm_b2 = model.sm_b * model.sm_b;

    double ep2 = (sm_a2 - sm_b2) / sm_b2;

    double cl  = cos(lat);
    double cl2 = cl  * cl;
    double cl3 = cl2 * cl;
    double cl4 = cl3 * cl;
    double cl5 = cl4 * cl;
    double cl6 = cl5 * cl;
    double cl7 = cl6 * cl;
    double cl8 = cl7 * cl;

    double lo  = lon - lon0;
    double lo2 = lo  * lo;
    double lo3 = lo2 * lo;
    double lo4 = lo3 * lo;
    double lo5 = lo4 * lo;
    double lo6 = lo5 * lo;
    double lo7 = lo6 * lo;
    double lo8 = lo7 * lo;

    double nu2 = ep2 * cl2;
    double N   = sm_a2 / (model.sm_b * sqrt(1.0 + nu2));
    double t   = tan(lat);
    double t2  = t*t;

    double l3  = 1.0 - t2 + nu2;
    double l4  = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
    double l5  = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
    double l6  = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
    double l7  = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
    double l8  = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    p.resize(2);

    p[0] = N * cl * lo
           + ((N / 6.0) * cl3 * l3 * lo3)
           + ((N / 120.0) * cl5 * l5 * lo5)
           + ((N / 5040.0) * cl7 * l7 * lo7);

    p[1] = arclen_of_a_meridian(lat)
           + (t / 2.0 * N * cl2 * lo2)
           + (t / 24.0 * N * cl4 * l4 * lo4)
           + (t / 720.0 * N * cl6 * l6 * lo6)
           + (t / 40320.0 * N * cl8 * l8 * lo8);

    return;
  }

  void xy_to_latlon(const Point& p, const double& lon0,
                    double& lat, double& lon,
                    const ModelConstants& model)
  {
    double sm_a2 = model.sm_a * model.sm_a;
    double sm_b2 = model.sm_b * model.sm_b;

    double lat_y = footpoint_lat(p[1], model);

    double ep2  = (sm_a2 - sm_b2) / sm_b2;
    double cly  = cos(lat_y);
    double nuy2 = ep2 * cly * cly;

    double Ny    = sm_a2 / (model.sm_b * sqrt(1.0 + nuy2));
    double Nypow = Ny;

    double ty   = tan(cly);
    double ty2  = ty * ty;
    double ty4  = ty2 * ty2;

    double x1   = 1.0 / (Nypow * cly);

    Nypow *= Ny;   /* now equals Ny**2) */
    double x2 = ty / (2.0 * Nypow);

    Nypow *= Ny;   /* now equals Ny**3) */
    double x3 = 1.0 / (6.0 * Nypow * cly);

    Nypow *= Ny;   /* now equals Ny**4) */
    double x4 = ty / (24.0 * Nypow);

    Nypow *= Ny;   /* now equals Ny**5) */
    double x5 = 1.0 / (120.0 * Nypow * cly);

    Nypow *= Ny;   /* now equals Ny**6) */
    double x6 = ty / (720.0 * Nypow);

    Nypow *= Ny;   /* now equals Ny**7) */
    double x7 = 1.0 / (5040.0 * Nypow * cly);

    Nypow *= Ny;   /* now equals Ny**8) */
    double x8 = ty / (40320.0 * Nypow);

    double x2_p = -1.0 - nuy2;
    double x3_p = -1.0 - 2 * ty2 - nuy2;
    double x4_p = 5.0 + 3.0 * ty2 + 6.0 * nuy2 - 6.0 * ty2 * nuy2 - 3.0 *
                  (nuy2 * nuy2) - 9.0 * ty2 * (nuy2 * nuy2);
    double x5_p = 5.0 + 28.0 * ty2 + 24.0 * ty4 + 6.0 * nuy2 + 8.0 * ty2 * nuy2;
    double x6_p = -61.0 - 90.0 * ty2 - 45.0 * ty4 - 107.0 * nuy2 + 162.0 * ty2 *
                  nuy2;
    double x7_p = -61.0 - 662.0 * ty2 - 1320.0 * ty4 - 720.0 * (ty4 * ty2);
    double x8_p = 1385.0 + 3633.0 * ty2 + 4095.0 * ty4 + 1575 * (ty4 * ty2);

    double x   = p[0];
    double x_2 = x*x;
    double x_3 = x_2*x;
    double x_4 = x_2*x_2;
    double x_5 = x_2*x_3;
    double x_6 = x_2*x_4;
    double x_7 = x_2*x_5;
    double x_8 = x_2*x_6;


    /* Calculate latitude */
    lat = lat_y + x2 * x2_p * x_2
          + x4 * x4_p * x_4
          + x6 * x6_p * x_6
          + x8 * x8_p * x_8;

    lat = Angle(lat, AngleType::rad).deg();

    /* Calculate longitude */
    lon = lon0 + x1 * x
          + x3 * x3_p * x_3
          + x5 * x5_p * x_5
          + x7 * x7_p * x_7;

    lon = Angle(lon, AngleType::rad).deg();
    return;
  }

  int compute_zone(const double& lon)
  {
    double l = (lon + 180.0) - floor((lon + 180.0) / 360.0) * 360.0 - 180.0;
    return int(floor((l + 180.0) / 6.0) + 1.0);
  }

  double compute_convergence_angle(const double& lat,
                                   const double& lon)
  {
//Meridiano centrale in radianti
    double lon_ = Angle(lon, AngleType::deg).rad();
    double lat_ = Angle(lat, AngleType::deg).rad();
    double lon0 = UTM_central_meridian(compute_zone(lon));
//es. zoneUTM Italia: 32 o 33
    return Angle(atan(tan(lon_ - lon0) * sin(lat_)), AngleType::rad).deg();
  }

  Point UTM_latlon_to_xy(const double& lat, const double& lon,
                         const ModelConstants& model)
  {
    int zone = compute_zone(lon);

    Point p({0.0, 0.0});

    latlon_to_xy(Angle(lat, AngleType::deg).rad(), Angle(lon, AngleType::deg).rad(),
                 UTM_central_meridian(zone), p);

    /* Adjust easting and northing for UTM system. */
    p[0] = p[0] * model.UTMScaleFactor + 500000.0;
    p[1] = p[1] * model.UTMScaleFactor;
    if(p[1] < 0.0)
      p[1] = p[1] + 10000000.0;

    return p;
  }

  void UTM_xy_to_latlon(const Point& p,
                        const int& zone, const bool& southhemi,
                        double& lat, double& lon,
                        const ModelConstants& model)
  {
    double x = p[0];
    double y = p[1];

    x -= 500000.0;
    x /= model.UTMScaleFactor;

    /* If in southern hemisphere, adjust y accordingly. */
    if(southhemi)
      y -= 10000000.0;

    y /= model.UTMScaleFactor;

    Point new_p({x,y});
    double cm = UTM_central_meridian(zone);
    xy_to_latlon(new_p, cm, lat, lon);
  }
}
