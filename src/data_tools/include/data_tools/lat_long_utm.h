#ifndef LAT_LONG_UTM_H
#define LAT_LONG_UTM_H

#include <string>
#include <tuple>

namespace lat_long_utm {

std::tuple<double, double, std::string> lat_long_to_UTM(const double Lat, const double Long);

} // namespace lat_long_utm

#endif // LAT_LONG_UTM_H
