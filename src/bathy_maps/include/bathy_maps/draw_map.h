/* Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DRAW_MAP_H
#define DRAW_MAP_H

#include <data_tools/std_data.h>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <opencv2/core/core.hpp>

class BathyMapImage {
private:
    cv::Point2f world_pos_to_image(const Eigen::Vector3d& pos, bool relative=false);
public:
    using TargetsT = std::map<std::string, std::pair<double, double> >;

    cv::Mat bathy_map;
    // res, xmin, ymin, imxmin, imymin
    std::array<double, 5> params;
    int rows, cols;

    BathyMapImage(const std_data::mbes_ping::PingsT& pings, int rows=500, int cols=500);
    BathyMapImage(const Eigen::MatrixXd& height_map, const Eigen::Matrix2d& bounds);
    void draw_track(const std_data::mbes_ping::PingsT& pings);
    void draw_track(const std_data::mbes_ping::PingsT& pings, const cv::Scalar& color);
    void draw_track(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& pos);
    void draw_height_map(const std_data::mbes_ping::PingsT& pings);
    void draw_height_map(const Eigen::MatrixXd& height_map);
    void draw_back_scatter_map(std_data::mbes_ping::PingsT& pings);
    void draw_targets(const TargetsT& targets, const cv::Scalar& color);
    void draw_indices(std_data::mbes_ping::PingsT& pings, int skip_indices=500);
    void draw_pose(const Eigen::Vector3d& pos, double heading, const cv::Scalar& color);
    void draw_red_pose(const Eigen::Vector3d& pos, double heading);
    void draw_blue_pose(const Eigen::Vector3d& pos, double heading);

    void rotate_crop_image(const Eigen::Vector3d& first_pos, const Eigen::Vector3d& last_pos, double result_width);
    void write_image(const boost::filesystem::path& path);
    void write_image_from_str(const std::string& path);
    cv::Mat make_image() { return bathy_map.clone(); }
    void show();
    void blip();
};

#endif // DRAW_MAP_H
