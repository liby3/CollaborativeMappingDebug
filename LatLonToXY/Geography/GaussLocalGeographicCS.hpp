#ifndef _GAUSSLOCALGEOGRAPHICCS_HPP_
#define _GAUSSLOCALGEOGRAPHICCS_HPP_

// copy from https://github.com/f03el/gauss-kruger-cpp.git
#include "private_hpps/gausskruger.hpp"
#include "LocalGeographicCSBase.hpp"

class GaussLocalGeographicCS: public LocalGeographicCSBase, public gausskruger::Projection {
    const double lat0;
    const double lon0;

    double x0;
    double y0;

    double flattening() { return 1 / 298.257222101; }
    double equatorialRadius() { return 6367558.4969; }

    double centralMeridian() { return lon0; }
    double scale() { return 1.0; }
    double falseNorthing() { return -y0; }
    double falseEasting() { return -x0; }

  public:
    GaussLocalGeographicCS(const double lat0, const double lon0) :
    	LocalGeographicCSBase(true), lat0(lat0), lon0(lon0), x0(0.0), y0(0.0) {
        geodeticToGrid(lat0, lon0, y0, x0);
    }

    virtual ~GaussLocalGeographicCS() {
    }

    virtual void llh2xyz(const double lat, const double lon, const double height,
                         double& x, double& y, double& z) {
        z = height;
        geodeticToGrid(lat, lon, y, x);
    }
    virtual void xyz2llh(const double x, const double y, const double z,
                         double& lat, double& lon, double& height) {
        height = z;
        gridToGeodetic(y, x, lat, lon);
    }
    virtual void subXYZOrigin(double& x, double& y, double& z) {
        x -= x0, y -= y0;
    }

};

#endif // _GAUSSLOCALGEOGRAPHICCS_HPP_
