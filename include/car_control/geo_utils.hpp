#pragma once

#include <cmath>

namespace geo
{

/**
 * Convert WGS-84 latitude/longitude to UTM zone 32N (EPSG:25832) easting/northing.
 *
 * Uses the standard Transverse Mercator series expansion.
 * Accuracy: better than 1 mm anywhere in UTM zone 32 (longitude 6°–18°E).
 *
 * @param lat_deg   Latitude  [degrees, WGS-84]
 * @param lon_deg   Longitude [degrees, WGS-84]
 * @param easting   Output: UTM32 easting  [metres, ~160 000 – 840 000]
 * @param northing  Output: UTM32 northing [metres, northern hemisphere 0 – 9 300 000]
 */
inline void latlon_to_utm32(
    double lat_deg, double lon_deg,
    double & easting, double & northing)
{
    // GRS80 ellipsoid (EPSG:25832 / EUREF89 datum)
    constexpr double a   = 6378137.0;
    constexpr double f   = 1.0 / 298.257222101;
    constexpr double e2  = 2.0 * f - f * f;          // first eccentricity squared
    constexpr double ep2 = e2 / (1.0 - e2);          // second eccentricity squared
    // UTM zone 32: central meridian 9 °E, scale factor 0.9996, false easting 500 000 m
    constexpr double k0   = 0.9996;
    constexpr double lon0 = 9.0 * M_PI / 180.0;
    constexpr double E0   = 500000.0;

    const double phi  = lat_deg * M_PI / 180.0;
    const double lam  = lon_deg * M_PI / 180.0;
    const double dlam = lam - lon0;

    const double sin_phi = std::sin(phi);
    const double cos_phi = std::cos(phi);
    const double tan_phi = std::tan(phi);

    const double N = a / std::sqrt(1.0 - e2 * sin_phi * sin_phi);
    const double T = tan_phi * tan_phi;
    const double C = ep2 * cos_phi * cos_phi;
    const double A = cos_phi * dlam;

    // Meridional arc
    const double e4 = e2 * e2;
    const double e6 = e4 * e2;
    const double M  = a * (
        (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0  - 5.0 * e6 / 256.0)  * phi
      - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0  + 45.0 * e6 / 1024.0) * std::sin(2.0 * phi)
      + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0)                  * std::sin(4.0 * phi)
      - (35.0 * e6 / 3072.0)                                        * std::sin(6.0 * phi));

    const double A2 = A * A;
    const double A3 = A2 * A;
    const double A4 = A2 * A2;
    const double A5 = A4 * A;
    const double A6 = A4 * A2;

    easting = k0 * N * (
        A
      + (1.0 - T + C) * A3 / 6.0
      + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A5 / 120.0)
      + E0;

    northing = k0 * (
        M
      + N * tan_phi * (
          A2 / 2.0
        + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0
        + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A6 / 720.0));
}

/**
 * Grid convergence γ at a WGS-84 point for UTM zone 32N: the signed angle from
 * grid (UTM) north to true north, in radians.  Positive east of the 9 °E central
 * meridian on the northern hemisphere; at Trondheim (~10.4 °E) γ ≈ +1.27°.
 *
 * Add γ to a true-north-referenced ENU yaw to express it in the UTM grid frame,
 * so a heading and a UTM position share one frame.  GRS80 ellipsoid, matching
 * latlon_to_utm32.  Series truncated after the third-order term: error < 1 µrad
 * anywhere in zone 32.
 *
 * @param lat_deg  Latitude  [degrees, WGS-84]
 * @param lon_deg  Longitude [degrees, WGS-84]
 * @return         Grid convergence γ [radians]
 */
inline double utm32_convergence(double lat_deg, double lon_deg)
{
    constexpr double a   = 6378137.0;
    constexpr double f   = 1.0 / 298.257222101;
    constexpr double e2  = 2.0 * f - f * f;
    constexpr double ep2 = e2 / (1.0 - e2);          // second eccentricity squared
    constexpr double lon0 = 9.0 * M_PI / 180.0;      // UTM zone 32 central meridian

    const double phi  = lat_deg * M_PI / 180.0;
    const double dlam = lon_deg * M_PI / 180.0 - lon0;

    const double sin_phi = std::sin(phi);
    const double cos_phi = std::cos(phi);
    const double C = ep2 * cos_phi * cos_phi;

    return dlam * sin_phi * (
        1.0
      + dlam * dlam / 3.0 * cos_phi * cos_phi * (1.0 + 3.0 * C + 2.0 * C * C));
}

/**
 * Legacy flat-Earth ENU helper (kept for reference; prefer latlon_to_utm32).
 */
inline void latlon_to_enu(
    double lat, double lon,
    double origin_lat, double origin_lon,
    double & east, double & north)
{
    constexpr double R = 6371000.0;
    const double lat_rad        = lat        * M_PI / 180.0;
    const double origin_lat_rad = origin_lat * M_PI / 180.0;
    east  = R * (lon - origin_lon) * M_PI / 180.0 * std::cos((lat_rad + origin_lat_rad) / 2.0);
    north = R * (lat - origin_lat) * M_PI / 180.0;
}

}  // namespace geo
