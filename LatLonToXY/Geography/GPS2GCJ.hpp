#ifndef _GPS2GCJ_HPP__
#define _GPS2GCJ_HPP__

#include <cmath>

namespace GPS2GCJ {
constexpr double eePara = 0.00669342162296594323;
constexpr double aaPara = 6378245.0;

inline double transformLat(const double x, const double y) {
	double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y
			+ 0.2 * sqrt(x > 0 ? x : -x);
	ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
	ret += (20.0 * sin(y * M_PI) + 40.0 * sin(y / 3.0 * M_PI)) * 2.0 / 3.0;
	ret += (160.0 * sin(y / 12.0 * M_PI) + 320 * sin(y * M_PI / 30.0)) * 2.0 / 3.0;

	return ret;
}

inline double transformLon(const double x, const double y) {
	double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(x > 0 ? x : -x);
	ret += (20.0 * sin(6.0 * x * M_PI) + 20.0 * sin(2.0 * x * M_PI)) * 2.0 / 3.0;
	ret += (20.0 * sin(x * M_PI) + 40.0 * sin(x / 3.0 * M_PI)) * 2.0 / 3.0;
	ret += (150.0 * sin(x / 12.0 * M_PI) + 300.0 * sin(x / 30.0 * M_PI)) * 2.0 / 3.0;

	return ret;
}

//call this funciton to get the corrected longitude and latitude
inline void transformWGS2GCJ(double& latitude, double& longitude) {
	double dLat = transformLat(longitude - 105.0, latitude - 35.0);
	double dLon = transformLon(longitude - 105.0, latitude - 35.0);
	double radLat = latitude / 180.0 * M_PI;
	double magic = sin(radLat);
	magic = 1 - eePara * magic * magic;
	double sqrtMagic = sqrt(magic);
	dLat = (dLat * 180.0) / ((aaPara * (1 - eePara)) / (magic * sqrtMagic) * M_PI);
	dLon = (dLon * 180.0) / (aaPara / sqrtMagic * cos(radLat) * M_PI);

	longitude = dLon + longitude;
	latitude = dLat + latitude;
}

}

#endif /*_GPS2GCJ_HPP__*/
