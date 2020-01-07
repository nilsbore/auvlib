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

#ifndef SSS_MAP_IMAGE_H
#define SSS_MAP_IMAGE_H

#include <Eigen/Dense>
#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>

#include <bathy_maps/patch_views.h>
#include <data_tools/std_data.h>

struct sss_map_image {

    using BoundsT = Eigen::Matrix2d;
    using ImagesT = std::vector<sss_map_image, Eigen::aligned_allocator<sss_map_image> >;

    BoundsT bounds;

    Eigen::MatrixXd sss_map_image_;

    double sss_ping_duration; // max time in waterfall image
    Eigen::MatrixXf sss_waterfall_image;
    Eigen::MatrixXf sss_waterfall_cross_track;
    Eigen::MatrixXf sss_waterfall_depth;
    Eigen::MatrixXf sss_waterfall_model;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pos;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(bounds), CEREAL_NVP(sss_map_image_), CEREAL_NVP(sss_ping_duration),
           CEREAL_NVP(sss_waterfall_image), CEREAL_NVP(sss_waterfall_cross_track),
           CEREAL_NVP(sss_waterfall_depth), CEREAL_NVP(sss_waterfall_model), CEREAL_NVP(pos));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class sss_map_image_builder {
public:

    using MapType = sss_map_image;

private:

    sss_map_image::BoundsT bounds;
    double resolution;

    Eigen::Vector3d global_origin;
    int image_rows;
    int image_cols;

    Eigen::MatrixXd sss_map_image_sums;
    Eigen::MatrixXd sss_map_image_counts;

    int waterfall_width;
    int waterfall_counter;
    double sss_ping_duration; // max time in waterfall image
    Eigen::MatrixXd sss_waterfall_image;
    Eigen::MatrixXd sss_waterfall_cross_track;
    Eigen::MatrixXd sss_waterfall_depth;
    Eigen::MatrixXd sss_waterfall_model;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > poss;

public:

    sss_map_image_builder(const sss_map_image::BoundsT& bounds, double resolution, int nbr_pings);

    std::pair<int, int> get_map_image_shape();

    size_t get_waterfall_bins();

    bool empty();

    Eigen::MatrixXd downsample_cols(const Eigen::MatrixXd& M, int new_cols);

    sss_map_image finish();

    void add_waterfall_images(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                              const std_data::sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left);

    void add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                  const std_data::sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left);

    void add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                  const Eigen::VectorXd& intensities,
                  const Eigen::VectorXd& sss_depths, const Eigen::VectorXd& sss_model,
                  const std_data::sss_ping_side& ping, const Eigen::Vector3d& pos,
                  const Eigen::Vector3d& rpy, bool is_left);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

sss_patch_views::ViewsT convert_maps_to_patches(const sss_map_image::ImagesT& map_images, const Eigen::MatrixXd& height_map, double patch_size);
sss_patch_views::ViewsT convert_maps_to_single_angle_patches(const sss_map_image::ImagesT& map_images, const Eigen::MatrixXd& height_map, double patch_size);

#endif // SSS_MAP_IMAGE_H
