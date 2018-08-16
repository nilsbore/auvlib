#ifndef DRAW_MAP_H
#define DRAW_MAP_H

#include <data_tools/data_structures.h>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>

class bathy_map_image {
public:
    using TargetsT = std::map<std::string, std::pair<double, double> >;

    cv::Mat bathy_map;
    // res, xmin, ymin, imxmin, imymin
    std::array<double, 5> params;
    int rows, cols;

    bathy_map_image(mbes_ping::PingsT& pings, int rows=500, int cols=500);
    void draw_track(mbes_ping::PingsT& pings, const cv::Scalar& color);
    void draw_height_map(mbes_ping::PingsT& pings);
    void draw_targets(const TargetsT& targets, const cv::Scalar& color);
    void save_image(const boost::filesystem::path& path);
};

#endif // DRAW_MAP_H
