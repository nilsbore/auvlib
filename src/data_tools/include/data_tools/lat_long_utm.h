#ifndef LAT_LONG_UTM_H
#define LAT_LONG_UTM_H

#include <string>
#include <tuple>

std::tuple<double, double, std::string> lat_long_to_UTM(const double Lat, const double Long);

#endif // LAT_LONG_UTM_H
