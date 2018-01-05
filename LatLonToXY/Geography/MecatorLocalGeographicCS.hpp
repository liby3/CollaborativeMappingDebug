// File:           MecatorLocalGeographicCS.hpp
// Creation Date:  Tuesday, March  6 2012
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#if !defined(LOCALGEOGRAPHICCS_HPP)
#define LOCALGEOGRAPHICCS_HPP

#include <utility>
#include "private_hpps/convert_coordinates.hpp"
#include "LocalGeographicCSBase.hpp"

struct MecatorLocalGeographicCS: public LocalGeographicCSBase {
    MecatorLocalGeographicCS(const double lat0, const double lon0);

    void set_origin(const double lat0, const double lon0);

    void ll2xy(const double lat, double lon, double& x, double& y);
    void xy2ll(const double x, double y, double& lat, double& lon);

    virtual void llh2xyz(const double lat, const double lon, const double height,
                         double& x, double& y, double& z) {
        z = height;
        ll2xy(lat, lon, x, y);
    }
    virtual void xyz2llh(const double x, const double y, const double z,
                         double& lat, double& lon, double& height) {
        height = z;
        xy2ll(x, y, lat, lon);
    }
    virtual void subXYZOrigin(double& x, double& y, double& z) {
        x -= _x0, y -= _y0;
    }

    std::pair<double, double> ll2xy(double lat, double lon);
    std::pair<double, double> xy2ll(double x, double y);

    // operate on containers
    template<class ItIn, class ItOut>
    void ll2xy(const ItIn & lat_begin, const ItIn & lat_end,
               const ItIn & lon_begin, const ItOut & x_begin,
               const ItOut & y_begin) const;

    template<class ItIn, class ItOut>
    void xy2ll(const ItIn & x_begin, const ItIn & x_end, const ItIn & y_begin,
               const ItOut & lat_begin, const ItOut & lon_begin) const;

  private:
    double _scale;
    double _x0, _y0;
};

inline MecatorLocalGeographicCS::MecatorLocalGeographicCS(const double lat0, const double lon0)
    : LocalGeographicCSBase(true) {
    set_origin(lat0, lon0);
}

inline void MecatorLocalGeographicCS::set_origin(const double lat0, const double lon0) {
    _scale = convert_coordinates::lat_to_scale(lat0);
    convert_coordinates::latlon_to_mercator(lat0, lon0, _scale, _x0, _y0);
}

inline void MecatorLocalGeographicCS::ll2xy(double lat, double lon, double& x,
        double& y) {
    convert_coordinates::latlon_to_mercator(lat, lon, _scale, x, y);
    x -= _x0;
    y -= _y0;
}

inline std::pair<double, double> MecatorLocalGeographicCS::ll2xy(double lat,
        double lon) {
    double x, y;
    ll2xy(lat, lon, x, y);
    return std::make_pair(x, y);
}

inline void MecatorLocalGeographicCS::xy2ll(double x, double y, double& lat,
        double& lon) {
    x += _x0;
    y += _y0;

    convert_coordinates::mercator_to_latlon(x, y, _scale, lat, lon);
}

inline std::pair<double, double> MecatorLocalGeographicCS::xy2ll(double x,
        double y) {
    double lat, lon;
    xy2ll(x, y, lat, lon);
    return std::make_pair(lat, lon);
}

// operate on containers
template<class ItIn, class ItOut>
void MecatorLocalGeographicCS::ll2xy(const ItIn& lat_begin, const ItIn& lat_end,
                                     const ItIn& lon_begin, const ItOut& x_begin,
                                     const ItOut& y_begin) const {
    ItIn lat = lat_begin;
    ItIn lon = lon_begin;
    ItOut x = x_begin;
    ItOut y = y_begin;

    for (; lat != lat_end; lat++, lon++, x++, y++) {
        ll2xy(*lat, *lon, *x, *y);
    }
}

template<class ItIn, class ItOut>
void MecatorLocalGeographicCS::xy2ll(const ItIn& x_begin, const ItIn& x_end,
                                     const ItIn& y_begin, const ItOut& lat_begin,
                                     const ItOut& lon_begin) const {
    ItIn x = x_begin;
    ItIn y = y_begin;

    ItOut lat = lat_begin;
    ItOut lon = lon_begin;

    for (; x != x_end; lat++, lon++, x++, y++)
        xy2ll(*x, *y, *lat, *lon);
}

#endif
