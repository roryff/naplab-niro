#pragma once

#include <cmath>

namespace geo
{

/**
 * Convert WGS-84 latitude/longitude to local ENU (East-North-Up) coordinates
 * using an equirectangular (flat-Earth) projection.
 *
 * Valid within roughly 10 km of the origin; error stays below ~0.1 % at that
 * range, which is sufficient for local path following.
 *
 * @param lat        Target latitude  [degrees]
 * @param lon        Target longitude [degrees]
 * @param origin_lat Origin latitude  [degrees]
 * @param origin_lon Origin longitude [degrees]
 * @param east       Output: East displacement  [metres]
 * @param north      Output: North displacement [metres]
 */
inline void latlon_to_enu(
    double lat, double lon,
    double origin_lat, double origin_lon,
    double & east, double & north)
{
    constexpr double R = 6371000.0;  // Earth mean radius [m]

    const double lat_rad        = lat        * M_PI / 180.0;
    const double origin_lat_rad = origin_lat * M_PI / 180.0;
    const double dLon           = (lon - origin_lon) * M_PI / 180.0;
    const double dLat           = (lat - origin_lat) * M_PI / 180.0;

    east  = R * dLon * std::cos((lat_rad + origin_lat_rad) / 2.0);
    north = R * dLat;
}

}  // namespace geo
