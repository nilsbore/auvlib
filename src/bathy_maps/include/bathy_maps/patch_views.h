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

#ifndef PATCH_VIEWS_H
#define PATCH_VIEWS_H

#include <Eigen/Dense>
#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>

struct sss_patch_views {

    using ViewsT = std::vector<sss_patch_views, Eigen::aligned_allocator<sss_patch_views> >;

    double patch_size;
    Eigen::Vector3d patch_origin;
    Eigen::MatrixXd patch_height;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > sss_views;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > patch_view_pos;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > patch_view_dirs;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(patch_height), CEREAL_NVP(sss_views), CEREAL_NVP(patch_height),
           CEREAL_NVP(sss_views), CEREAL_NVP(patch_view_pos), CEREAL_NVP(patch_view_dirs));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class sss_patch_assembler {

private:

    int image_size;
    double world_size;

    bool is_active_;
    Eigen::Vector3d origin;

    Eigen::Vector3d current_pos_value;
    Eigen::Vector3d first_pos;
    int current_pos_count;
    Eigen::MatrixXd current_view_value;
    Eigen::MatrixXd current_view_count;

    Eigen::MatrixXd height_value;
    Eigen::MatrixXd height_count;

    sss_patch_views patch_views;

    // DEBUG
    int nbr_splits;

public:

    sss_patch_assembler(int image_size=30, double world_size=8.);

    double get_world_size() const { return world_size; }
    Eigen::Vector3d get_origin() const { return origin; }
    Eigen::MatrixXd get_last_patch_view() const { return patch_views.sss_views.back(); };

    bool empty();

    void activate(const Eigen::Vector3d& new_origin);

    bool is_active();

    void split();

    sss_patch_views finish();

    //void add_hits(const Eigen::MatrixXd& hits, const Eigen::Vector3d& pos);
    void add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXd& intensities, const Eigen::Vector3d& pos);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif // PATCH_VIEWS_H
