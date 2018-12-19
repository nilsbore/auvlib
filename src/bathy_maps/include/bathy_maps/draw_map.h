#ifndef DRAW_MAP_H
#define DRAW_MAP_H

#include <data_tools/data_structures.h>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>

class BathyMapImage {
public:
    using TargetsT = std::map<std::string, std::pair<double, double> >;

    cv::Mat bathy_map;
    // res, xmin, ymin, imxmin, imymin
    std::array<double, 5> params;
    int rows, cols;

    BathyMapImage(data_structures::mbes_ping::PingsT& pings, int rows=500, int cols=500);
    void draw_track(data_structures::mbes_ping::PingsT& pings);
    void draw_track(data_structures::mbes_ping::PingsT& pings, const cv::Scalar& color);
    void draw_height_map(data_structures::mbes_ping::PingsT& pings);
    void draw_back_scatter_map(data_structures::mbes_ping::PingsT& pings);
    void draw_targets(const TargetsT& targets, const cv::Scalar& color);
    void write_image(const boost::filesystem::path& path);
    void write_image_from_str(const std::string& path);
    void show();
};

#endif // DRAW_MAP_H
