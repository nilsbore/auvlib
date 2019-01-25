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

#ifndef SSS_GEN_SIM_H
#define SSS_GEN_SIM_H

#include <bathy_maps/base_draper.h>
#include <bathy_maps/patch_views.h>
#include <bathy_maps/sss_map_image.h>
#include <opencv2/core/core.hpp>

struct SSSGenSim : public BaseDraper {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    BoundsT bounds;
    //double resolution;
    std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)> gen_callback;
    Eigen::MatrixXd height_map;
    cv::Mat height_map_cv;
    Eigen::MatrixXd gen_sss_window;
    Eigen::Vector3d window_point;
    double window_heading;
    cv::Mat waterfall_image;
    cv::Mat gt_waterfall_image;
    Eigen::MatrixXd texture;
    size_t nbr_windows;

    void generate_sss_window();
    Eigen::VectorXd compute_times(const Eigen::MatrixXd& P);
    Eigen::VectorXd compute_time_windows(const Eigen::VectorXd& times, const Eigen::VectorXd& intensities, const xtf_data::xtf_sss_ping_side& ping);
    void visualize_rays(const Eigen::MatrixXd& hits_left, const Eigen::MatrixXd& hits_right);
    void visualize_vehicle();
    Eigen::VectorXd get_texture_intensities(const Eigen::MatrixXd& P);
    Eigen::MatrixXd get_UV(const Eigen::MatrixXd& P);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> project();

public:
    
    static Eigen::MatrixXd default_callback(const Eigen::MatrixXd& window) { return window; }

    void set_gen_callback(const std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)>& callback) { gen_callback = callback; }
    //void set_resolution(double new_resolution);

    SSSGenSim(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
              const xtf_data::xtf_sss_ping::PingsT& pings,
              const BoundsT& bounds,
              const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds,
              const Eigen::MatrixXd& height_map);

    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
};

/*
sss_map_image::ImagesT drape_maps(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                  const SSSGenSim::BoundsT& bounds, const xtf_data::xtf_sss_ping::PingsT& pings,
                                  const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                  double resolution, const std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)>& gen_callback);
*/

#endif // SSS_GEN_SIM_H
