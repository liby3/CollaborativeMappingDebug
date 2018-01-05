//          Copyright Erik Lundin 2016.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// Version: 1.0.0

#ifndef GAUSSKRUGER_HPP
#define GAUSSKRUGER_HPP

#include <cmath>

namespace gausskruger {

class Projection {
  public:
    Projection() {}
    virtual ~Projection() {}
    virtual double centralMeridian() = 0;
    virtual double flattening() = 0;
    virtual double equatorialRadius() = 0;
    virtual double scale() = 0;
    virtual double falseNorthing() = 0;
    virtual double falseEasting() = 0;

    void geodeticToGrid(double latitude, double longitude, double& northing, double& easting) {
        const double e2 = flattening() * (2 - flattening()); // e2: first eccentricity squared
        const double n = flattening() / (2 - flattening()); // n: 3rd flattening
        const double rectifyingRadius = equatorialRadius() / (1 + n) * (1 + 0.25 * pow(n, 2) + 0.015625 * pow(n, 4));

        double A = e2;
        double B = (5 * pow(e2, 2) - pow(e2, 3)) / 6.0;
        double C = (104 * pow(e2, 3) - 45 * pow(e2, 4)) / 120.0;
        double D = (1237 * pow(e2, 4)) / 1260.0;

        // Latitude and longitude are expected to be given in degrees
        // phi and lambda: latitude and longitude in radians
        double phi = latitude * M_PI / 180;
        double lambda = longitude * M_PI / 180;
        double lambda0 = centralMeridian() * M_PI / 180;

        // deltaLambda: longitude relative to the central meridian
        double deltaLambda = lambda - lambda0;

        // phiStar: conformal latitude
        double phiStar =
            phi - sin(phi) * cos(phi) *
            (A + B * pow(sin(phi), 2) + C * pow(sin(phi), 4) + D * pow(sin(phi), 6));

        double xiPrim = atan(tan(phiStar) / cos(deltaLambda));
        double etaPrim = atanh(cos(phiStar) * sin(deltaLambda));

        double beta1 = 1 / 2.0 * n - 2 / 3.0 * pow(n, 2) + 5 / 16.0 * pow(n, 3)   + 41 / 180.0 * pow(n, 4);
        double beta2 =             13 / 48.0 * pow(n, 2)  - 3 / 5.0 * pow(n, 3) + 557 / 1440.0 * pow(n, 4);
        double beta3 =                                   61 / 240.0 * pow(n, 3)  - 103 / 140.0 * pow(n, 4);
        double beta4 =                                                        49561 / 161280.0 * pow(n, 4);

        northing = falseNorthing()
                   + scale() * rectifyingRadius * (xiPrim
                           + beta1 * sin(2 * xiPrim) * cosh(2 * etaPrim)
                           + beta2 * sin(4 * xiPrim) * cosh(4 * etaPrim)
                           + beta3 * sin(6 * xiPrim) * cosh(6 * etaPrim)
                           + beta4 * sin(8 * xiPrim) * cosh(8 * etaPrim));
        easting = falseEasting()
                  + scale() * rectifyingRadius * (etaPrim
                          + beta1 * cos(2 * xiPrim) * sinh(2 * etaPrim)
                          + beta2 * cos(4 * xiPrim) * sinh(4 * etaPrim)
                          + beta3 * cos(6 * xiPrim) * sinh(6 * etaPrim)
                          + beta4 * cos(8 * xiPrim) * sinh(8 * etaPrim));
    }

    void gridToGeodetic(double northing, double easting, double& latitude, double& longitude) {
        const double e2 = flattening() * (2 - flattening()); // e2: first eccentricity squared
        const double n = flattening() / (2 - flattening()); // n: 3rd flattening
        const double rectifyingRadius = equatorialRadius() / (1 + n) * (1 + 0.25 * pow(n, 2) + 0.015625 * pow(n, 4));
        double xi = (northing - falseNorthing()) / (scale() * rectifyingRadius);
        double eta = (easting - falseEasting()) / (scale() * rectifyingRadius);

        double delta1 = 1 / 2.0 * n - 2 / 3.0 * pow(n, 2) + 37 / 96.0 * pow(n, 3)     - 1 / 360.0 * pow(n, 4);
        double delta2 =              1 / 48.0 * pow(n, 2)  + 1 / 15.0 * pow(n, 3)  - 437 / 1440.0 * pow(n, 4);
        double delta3 =                                    17 / 480.0 * pow(n, 3)    - 37 / 840.0 * pow(n, 4);
        double delta4 =                                                           4397 / 161280.0 * pow(n, 4);

        double xiPrim = xi
                        - delta1 * sin(2 * xi) * cosh(2 * eta)
                        - delta2 * sin(4 * xi) * cosh(4 * eta)
                        - delta3 * sin(6 * xi) * cosh(6 * eta)
                        - delta4 * sin(8 * xi) * cosh(8 * eta);
        double etaPrim = eta
                         - delta1 * cos(2 * xi) * sinh(2 * eta)
                         - delta2 * cos(4 * xi) * sinh(4 * eta)
                         - delta3 * cos(6 * xi) * sinh(6 * eta)
                         - delta4 * cos(8 * xi) * sinh(8 * eta);

        double phiStar = asin(sin(xiPrim) / cosh(etaPrim)); // Conformal latitude
        double deltaLambda = atan(sinh(etaPrim) / cos(xiPrim));

        double AStar =  e2     + pow(e2, 2)       + pow(e2, 3)        + pow(e2, 4);
        double BStar =      (7 * pow(e2, 2)  + 17 * pow(e2, 3)   + 30 * pow(e2, 4)) / -6;
        double CStar =                       (224 * pow(e2, 3)  + 889 * pow(e2, 4)) / 120;
        double DStar =                                          (4279 * pow(e2, 4)) / -1260;

        double phi = phiStar
                     + sin(phiStar) * cos(phiStar) * (  AStar
                             + BStar * pow(sin(phiStar), 2)
                             + CStar * pow(sin(phiStar), 4)
                             + DStar * pow(sin(phiStar), 6));

        // phi: latitude in radians, lambda: longitude in radians
        // Return latitude and longitude as degrees
        latitude = phi * 180 / M_PI;
        longitude = centralMeridian() + deltaLambda * 180 / M_PI;
    }

};

} // namespace gausskruger

#endif // GAUSSKRUGER_HPP
