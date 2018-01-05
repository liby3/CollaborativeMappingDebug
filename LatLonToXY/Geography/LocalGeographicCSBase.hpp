#ifndef _LOCALGEOGRAPHICCSBASE_HPP__
#define _LOCALGEOGRAPHICCSBASE_HPP__

struct LocalGeographicCSBase {
    const bool isDescartesCoordinates;

    virtual void llh2xyz(const double lat, const double lon, const double height,
                         double& x, double& y, double& z) = 0;
    virtual void xyz2llh(const double x, const double y, const double z,
                         double& lat, double& lon, double& height) = 0;
    virtual void subXYZOrigin(double& x, double& y, double& z) = 0;

    LocalGeographicCSBase(bool isDescartesCoordinates):
        isDescartesCoordinates(isDescartesCoordinates) {}

    virtual ~LocalGeographicCSBase() {
    }
};

#endif /*_LOCALGEOGRAPHICCSBASE_HPP__*/