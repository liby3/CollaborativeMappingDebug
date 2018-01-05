#ifndef _ECEFLOCALGEOGRAPHICCS_HPP_
#define _ECEFLOCALGEOGRAPHICCS_HPP_

#include <cmath>
#include "LocalGeographicCSBase.hpp"

// fork from https://github.com/MoriokaReimen/GPS
class ECEFLocalGeographicCS: public LocalGeographicCSBase {
    constexpr static double toRad(const double deg) { return (deg) * (M_PI / 180.0); }
    constexpr static double toDeg(const double rad) { return (rad) * (180.0 / M_PI); }

    constexpr static double A = 6378137.0; /**< Equational Radius [m] */
    constexpr static double B = 6356752.3142; /**< Polar Radius [m]  */
    constexpr static double F = 1.0 / 298.257223563; /**< flattening */
    constexpr static double E = sqrt((A * A - B * B) / (A * A)); /**< Eccentricity */
    constexpr static double Er = sqrt((A * A - B * B) / (B * B)); /**< Second Eccentricity */

    const double lat0;
    const double lon0;

    double x0;
    double y0;
    double z0;

  public:
    ECEFLocalGeographicCS(const double lat0, const double lon0)  :
    	LocalGeographicCSBase(false), lat0(lat0), lon0(lon0), x0(0.0), y0(0.0), z0(0.0) {
        llh2xyz(lat0, lon0, 0.0, y0, x0, z0);
    }

    virtual ~ECEFLocalGeographicCS() {
    }

    virtual void llh2xyz(const double lat, const double lon, const double height,
                         double& x, double& y, double& z) {
        const double n = A / sqrt(1.0 - E * E * sin(toRad(lat)) * sin(toRad(lat)));

        x = (n + height) * cos(toRad(lon)) * cos(toRad(lat)) - x0;
        y = (n + height) * sin(toRad(lon)) * cos(toRad(lat)) - y0;
        z = ((n * (1.0 - E * E)) + height) * sin(toRad(lat)) - z0;
    }

    virtual void xyz2llh(const double x, const double y, const double z,
                         double& lat, double& lon, double& height) {
        const double my_x = x + x0;
        const double my_y = y + y0;
        const double my_z = z + z0;

        double p = sqrt((my_x * my_x) + (my_y * my_y));
        double theta = atan2((my_z * A), (p * B));

        /* Avoid 0 division error */
        if (my_x == 0.0 && my_y == 0.0) {
            lon = 0.0, lat = copysign(90.0, my_z), height = my_z - copysign(B, my_z);
            return;
        } else {
            lat = atan2(
                      (my_z + (Er * Er * B * pow(sin(theta), 3))),
                      (p - (E * E * A * pow(cos(theta), 3)))
                  );
            lon = atan2(my_y, my_x);
            double n = A / sqrt(1.0 - E * E * sin(lat) * sin(lat));
            height = p / cos(lat) - n;

            lon = toDeg(lon), lat = toDeg(lat);
            return;
        }
    }

    virtual void subXYZOrigin(double& x, double& y, double& z) {
        x -= x0, y -= y0, z -= z0;
    }

};

#endif // _ECEFLOCALGEOGRAPHICCS_HPP_
