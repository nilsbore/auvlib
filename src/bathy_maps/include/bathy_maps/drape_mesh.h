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

#ifndef DRAPE_MESH_H
#define DRAPE_MESH_H

#include <Eigen/Dense>
#include <data_tools/xtf_data.h>
#include <data_tools/csv_data.h>

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> compute_sss_dirs(const Eigen::Matrix3d& R, double tilt_angle, double beam_width, int nbr_lines);

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi> compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width, const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1);

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXd, Eigen::VectorXd> embree_compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width, const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1);

/*
std::pair<Eigen::MatrixXd, Eigen::VectorXi> correlate_hits(const Eigen::MatrixXd& hits_port,
                                            const Eigen::VectorXi& hits_port_inds,
                                            const Eigen::VectorXd& mod_port,
                                            const xtf_data::xtf_sss_ping_side& ping,
                                            const Eigen::Vector3d& origin,
                                            double sound_vel,
                                            const Eigen::MatrixXi& F1,
                                            const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds,
                                            bool sound_speed_layers,
                                            Eigen::MatrixXd& C,
                                            Eigen::VectorXd& hit_sums,
                                            Eigen::VectorXi& hit_counts,
                                            bool is_left = false);
*/

bool point_in_view(const xtf_data::xtf_sss_ping& ping, const Eigen::Vector3d& point, double sensor_yaw);

bool is_mesh_underneath_vehicle(const Eigen::Vector3d& origin, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
double depth_mesh_underneath_vehicle(const Eigen::Vector3d& origin, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);


#endif // DRAPE_MESH_H
