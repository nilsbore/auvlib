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

#ifndef SSS_MEAS_DATA_H
#define SSS_MEAS_DATA_H

#include <Eigen/Dense>
#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <data_tools/std_data.h>

struct sss_meas_data {

    using ImagesT = std::vector<sss_meas_data, Eigen::aligned_allocator<sss_meas_data> >;
    using BoundsT = Eigen::Matrix2d;

    Eigen::MatrixXf sss_waterfall_image;
    Eigen::MatrixXf sss_waterfall_hits_X;
    Eigen::MatrixXf sss_waterfall_hits_Y;
    Eigen::MatrixXf sss_waterfall_hits_Z;

    std::vector<int> ping_id;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pos;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rpy;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(sss_waterfall_image), CEREAL_NVP(sss_waterfall_hits_X),
           CEREAL_NVP(sss_waterfall_hits_Y), CEREAL_NVP(sss_waterfall_hits_Z),
           CEREAL_NVP(ping_id), CEREAL_NVP(pos), CEREAL_NVP(rpy));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class sss_meas_data_builder {
public:

    using MapType = sss_meas_data;

private:

    Eigen::Vector3d global_origin;

    int waterfall_width;
    int waterfall_counter;
    Eigen::MatrixXd sss_waterfall_image;
    Eigen::MatrixXd sss_waterfall_hits_X;
    Eigen::MatrixXd sss_waterfall_hits_Y;
    Eigen::MatrixXd sss_waterfall_hits_Z;

    std::vector<int> ping_id;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pos;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rpy;

public:

    sss_meas_data_builder(const sss_meas_data::BoundsT& bounds, double resolution, int nbr_pings); // used

    size_t get_waterfall_bins(); // used

    bool empty(); // used

    Eigen::MatrixXd downsample_cols(const Eigen::MatrixXd& M, int new_cols);

    sss_meas_data finish(); // used

    void add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                  const Eigen::VectorXd& intensities,
                  const Eigen::VectorXd& sss_depths, const Eigen::VectorXd& sss_model,
                  const std_data::sss_ping_side& ping, const Eigen::Vector3d& pos,
                  const Eigen::Vector3d& rpy, bool is_left); // used

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif // SSS_MEAS_DATA_H
