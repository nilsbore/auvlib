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

#ifndef BASE_DRAPER_H
#define BASE_DRAPER_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>

#include <eigen3/Eigen/Dense>

#include <data_tools/xtf_data.h>
#include <data_tools/csv_data.h>

struct BaseDraper {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    igl::opengl::glfw::Viewer viewer;
    xtf_data::xtf_sss_ping::PingsT pings;
    int i;
    Eigen::MatrixXd V1;
    Eigen::MatrixXi F1;
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    Eigen::Vector3d offset;
    Eigen::VectorXd hit_sums;
    Eigen::VectorXi hit_counts;
    Eigen::MatrixXd N_faces; // the normals of F1, V1
    csv_data::csv_asvp_sound_speed::EntriesT sound_speeds;
    double sensor_yaw;
    bool ray_tracing_enabled; // is snell ray tracing enabled?

    // NOTE: these are new style functions

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> project();

    Eigen::VectorXd compute_times(const Eigen::MatrixXd& P);

    Eigen::VectorXd convert_to_time_bins(const Eigen::VectorXd& times, const Eigen::VectorXd& values,
                                         const xtf_data::xtf_sss_ping_side& ping, size_t nbr_windows);

    Eigen::VectorXd compute_intensities(const Eigen::VectorXd& times, 
                                        const xtf_data::xtf_sss_ping_side& ping);

    void visualize_rays(const Eigen::MatrixXd& hits_left, const Eigen::MatrixXd& hits_right);

    void visualize_vehicle();

    Eigen::VectorXd compute_model_intensities(const Eigen::MatrixXd& hits, const Eigen::MatrixXd& normals,
                                              const Eigen::Vector3d& origin);

    // NOTE: these are old style functions, to be deprecated
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXi, Eigen::VectorXi, Eigen::Vector3d> project_sss();

public:

    void set_texture(const Eigen::MatrixXd& texture, const BoundsT& bounds);
    void set_sidescan_yaw(double new_sensor_yaw) { sensor_yaw = new_sensor_yaw; }
    void set_ray_tracing_enabled(bool enabled);
    void set_vehicle_mesh(const Eigen::MatrixXd& new_V2, const Eigen::MatrixXi& new_F2, const Eigen::MatrixXd& new_C2);

    BaseDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
               const xtf_data::xtf_sss_ping::PingsT& pings,
               const BoundsT& bounds,
               const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds = csv_data::csv_asvp_sound_speed::EntriesT());

    void show();
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
};

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd> get_vehicle_mesh();
Eigen::MatrixXd color_jet_from_mesh(const Eigen::MatrixXd& V);
void drape_viewer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                  const BaseDraper::BoundsT& bounds, const xtf_data::xtf_sss_ping::PingsT& pings,
                  const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw);

#endif // BASE_DRAPER_H
