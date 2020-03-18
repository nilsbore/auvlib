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

#include <bathy_maps/view_draper.h>
#include <bathy_maps/patch_views.h>
#include <bathy_maps/sss_map_image.h>
#include <opencv2/core/core.hpp>

class sss_window_views
{
private:

    Eigen::MatrixXd sss_window;
    Eigen::MatrixXd depth_window;

    Eigen::Vector3d pos;
    double heading;
    int counter;

    Eigen::MatrixXd get_UV(const Eigen::MatrixXd& P)
    {
        // we need to specify this in the constructor!
        double resolution;// = double(height_map_cv.cols)/(bounds(1, 0) - bounds(0, 0));
        double heading = heading;
        Eigen::MatrixXd UV = P.leftCols<2>();
        UV.array().rowwise() -= pos.head<2>().transpose().array();
        Eigen::Matrix2d R = Eigen::AngleAxisd(-heading, Eigen::Vector3d::UnitZ()).matrix().topLeftCorner<2, 2>();
        UV *= R.transpose();
        UV.col(0).array() /= 1./resolution*double(sss_window.rows());
        UV.col(1).array() /= 1./resolution*double(sss_window.cols());
        UV.array().rowwise() += 0.5*Eigen::RowVector2d::Ones().array();
        return UV;
    }

public:

    sss_window_views(const Eigen::Vector3d& pos, double heading, int window_len, int window_width=512) : pos(pos), heading(heading)
    {
        sss_window = Eigen::MatrixXd::Zero(window_len, window_width);
        depth_window = Eigen::MatrixXd::Zero(window_len, window_width);
    }

    void add_swath(const Eigen::MatrixXd& hits, const Eigen::VectorXd& intensities)
    {
        Eigen::MatrixXd UV = get_UV(hits);
        for (int i = 0; i < UV.rows(); ++i) {
            if (UV(i, 0) < 1. && UV(i, 0) > 0. && UV(i, 1) < 1. && UV(i, 1) > 0.) {
                int row = int(double(sss_window.rows())*UV(i, 0));
                int col = int(double(sss_window.cols())*UV(i, 1));
                sss_window(row, col) = intensities(i);
                depth_window(row, col) = hits(i, 2);
            }
            else {
                std::cout << "Texture coordinates not correct: " << UV(i, 0) << ", " << UV(i, 1) << std::endl;
            }
        }
    }

    bool is_finished(const Eigen::Vector3d& current_pos, double current_heading)
    {

    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

struct SSSGenSim : public ViewDraper {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    bool sss_from_waterfall;
    bool sss_from_bathy;

    //BoundsT bounds;
    //double resolution;
    std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)> gen_callback;
    Eigen::MatrixXd height_map;
    cv::Mat height_map_cv;
    //Eigen::MatrixXd gen_sss_window;

    Eigen::Vector3d window_point;
    double window_heading;
    Eigen::MatrixXd texture;
    size_t nbr_windows;

    cv::Mat waterfall_image;
    cv::Mat gt_waterfall_image;
    cv::Mat model_waterfall_image;
    Eigen::MatrixXd waterfall_depth;
    Eigen::MatrixXd waterfall_model;
    size_t waterfall_row;
    size_t resample_window_height;
    size_t full_window_height;

    double left_row_mean;
    double right_row_mean;

    void generate_sss_window();
    //Eigen::VectorXd compute_times(const Eigen::MatrixXd& P);
    //Eigen::VectorXd compute_model_intensities(const Eigen::MatrixXd& hits, const Eigen::MatrixXd& normals,
    //                                          const Eigen::Vector3d& origin);
    //void visualize_rays(const Eigen::MatrixXd& hits_left, const Eigen::MatrixXd& hits_right);
    //void visualize_vehicle();
    Eigen::VectorXd get_texture_intensities(const Eigen::MatrixXd& P);
    Eigen::MatrixXd get_UV(const Eigen::MatrixXd& P);
    //std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> project();
    void construct_gt_waterfall();
    void construct_model_waterfall(const Eigen::MatrixXd& hits_left, const Eigen::MatrixXd& hits_right,
                                   const Eigen::MatrixXd& normals_left, const Eigen::MatrixXd& normals_right,
                                   const Eigen::VectorXd& times_left, const Eigen::VectorXd& times_right);

public:

    void set_gen_window_height(int gen_window_height)
    {
        full_window_height = size_t(gen_window_height);
        waterfall_depth = Eigen::MatrixXd::Zero(full_window_height, 2*nbr_windows);
    }

    void set_sss_from_waterfall(bool wf) { sss_from_waterfall = wf; };
    void set_sss_from_bathy(bool bathy) { sss_from_bathy = bathy; };

    Eigen::MatrixXd draw_sim_waterfall(const Eigen::MatrixXd& incidence_image);
    Eigen::MatrixXd draw_model_waterfall(const Eigen::MatrixXd& incidence_image, double sss_ping_duration);
    
    static Eigen::MatrixXd default_callback(const Eigen::MatrixXd& window) { return window; }

    void set_gen_callback(const std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)>& callback) { gen_callback = callback; }
    //void set_resolution(double new_resolution);

    SSSGenSim(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
              const std_data::sss_ping::PingsT& pings,
              const BoundsT& bounds,
              const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds,
              const Eigen::MatrixXd& height_map);

    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods);
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
};

#endif // SSS_GEN_SIM_H
