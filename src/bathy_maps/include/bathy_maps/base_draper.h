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

#include <Eigen/Dense>
#include <random>

#include <data_tools/xtf_data.h>
#include <data_tools/csv_data.h>

struct ping_draping_result {
    Eigen::Vector3d origin;
    Eigen::MatrixXd hits;
    Eigen::VectorXi hits_inds;
    Eigen::VectorXd times;
    Eigen::VectorXd intensities;

    //Eigen::VectorXd sss_depths;
    Eigen::MatrixXd sss_hits;
    Eigen::MatrixXd sss_normals;
    Eigen::VectorXd sss_model;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(origin), CEREAL_NVP(hits),
           CEREAL_NVP(hits_inds), CEREAL_NVP(hits_inds),
           CEREAL_NVP(intensities), CEREAL_NVP(sss_hits),
           CEREAL_NVP(sss_normals), CEREAL_NVP(sss_model));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct BaseDraper {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    Eigen::MatrixXd V1; // bathymetry mesh faces
    Eigen::MatrixXi F1; // bathymetry mesh vertices
    Eigen::Vector3d offset; // offset of mesh wrt world coordinates

    // smaller local versions used for ray tracing
    Eigen::MatrixXd V1_small; // bathymetry mesh faces
    Eigen::MatrixXi F1_small; // bathymetry mesh vertices
    Eigen::MatrixXd N_small; // normals of V1_small, F1_small
    Eigen::Vector3d pos_small; // the pos of the local small grid

    BoundsT bounds;
    Eigen::MatrixXd texture_image; // used for displaying texture and checking coverage

    //Eigen::VectorXd hit_sums; 
    //Eigen::VectorXi hit_counts;
    //Eigen::MatrixXd N_faces; // the normals of F1, V1, i.e. the bathymetry mesh
    csv_data::csv_asvp_sound_speed::EntriesT sound_speeds;
    double sensor_yaw;
    Eigen::Vector3d sensor_offset_port;
    Eigen::Vector3d sensor_offset_stbd;
    bool ray_tracing_enabled; // is snell ray tracing enabled?
    double tracing_map_size;
    double intensity_multiplier;
    std::default_random_engine generator; // hopefully not same seed every time

    // NOTE: these are new style functions

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> project(const xtf_data::xtf_sss_ping& ping);

    ping_draping_result project_ping_side(const xtf_data::xtf_sss_ping_side& sensor, const Eigen::MatrixXd& hits,
                                          const Eigen::MatrixXd& hits_normals, const Eigen::Vector3d& origin,
                                          int nbr_bins);

    double compute_simple_sound_vel();

    std::pair<Eigen::Vector3d, Eigen::Vector3d> get_port_stbd_sensor_origins(const xtf_data::xtf_sss_ping& ping);

    Eigen::VectorXd compute_times(const Eigen::Vector3d& sensor_origin, const Eigen::MatrixXd& P);

    Eigen::VectorXi compute_bin_indices(const Eigen::VectorXd& times, const xtf_data::xtf_sss_ping_side& ping, size_t nbr_windows);

    Eigen::VectorXd convert_to_time_bins(const Eigen::VectorXd& times, const Eigen::VectorXd& values,
                                         const xtf_data::xtf_sss_ping_side& ping, size_t nbr_windows);
    Eigen::MatrixXd convert_to_time_bins(const Eigen::VectorXd& times, const Eigen::MatrixXd& values,
                                         const xtf_data::xtf_sss_ping_side& ping, size_t nbr_windows);

    Eigen::VectorXd compute_intensities(const Eigen::VectorXd& times, 
                                        const xtf_data::xtf_sss_ping_side& ping);

    Eigen::VectorXd compute_lambert_intensities(const Eigen::MatrixXd& hits, const Eigen::MatrixXd& normals,
                                                const Eigen::Vector3d& origin);

    Eigen::VectorXd compute_model_intensities(const Eigen::VectorXd& dists, const Eigen::VectorXd& thetas);
    Eigen::VectorXd compute_model_intensities(const Eigen::MatrixXd& hits, const Eigen::MatrixXd& normals,
                                              const Eigen::Vector3d& origin);

    // NOTE: these are old style functions, to be deprecated
    //std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXi, Eigen::VectorXi, Eigen::Vector3d> project_sss();
    bool fast_is_mesh_underneath_vehicle(const Eigen::Vector3d& origin);

public:

    BaseDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
               const BoundsT& bounds,
               const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds = csv_data::csv_asvp_sound_speed::EntriesT());

    std::pair<ping_draping_result, ping_draping_result> project_ping(const xtf_data::xtf_sss_ping& ping, int nbr_bins);
    void add_texture_intensities(const Eigen::MatrixXd& hits, const Eigen::VectorXd& intensities);
    Eigen::VectorXd compute_bin_intensities(const xtf_data::xtf_sss_ping_side& ping, int nbr_bins);

    Eigen::MatrixXd get_texture_image() { return texture_image; }
    void set_sidescan_yaw(double new_sensor_yaw) { sensor_yaw = new_sensor_yaw; }
    void set_sidescan_port_stbd_offsets(const Eigen::Vector3d& new_offset_port, const Eigen::Vector3d& new_offset_stbd) { sensor_offset_port = new_offset_port; sensor_offset_stbd = new_offset_stbd; }
    void set_tracing_map_size(double new_tracing_map_size) { tracing_map_size = new_tracing_map_size; }
    void set_intensity_multiplier(double new_intensity_multiplier) { intensity_multiplier = new_intensity_multiplier; }
    void set_ray_tracing_enabled(bool enabled);

};


#endif // BASE_DRAPER_H
