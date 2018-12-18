#ifndef SSS_MAP_IMAGE_H
#define SSS_MAP_IMAGE_H

#include <eigen3/Eigen/Dense>
#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>

#include <bathy_maps/patch_views.h>
#include <data_tools/xtf_data.h>

struct sss_map_image {

    using BoundsT = Eigen::Matrix2d;
    using ImagesT = std::vector<sss_map_image, Eigen::aligned_allocator<sss_map_image> >;

    BoundsT bounds;

    Eigen::MatrixXd sss_map_image;

    Eigen::MatrixXf sss_waterfall_image;
    Eigen::MatrixXf sss_waterfall_cross_track;
    Eigen::MatrixXf sss_waterfall_depth;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pos;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(bounds), CEREAL_NVP(sss_map_image), CEREAL_NVP(sss_waterfall_image),
           CEREAL_NVP(sss_waterfall_cross_track), CEREAL_NVP(sss_waterfall_depth),
           CEREAL_NVP(pos));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class sss_map_image_builder {

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
    Eigen::MatrixXd sss_waterfall_image;
    Eigen::MatrixXd sss_waterfall_cross_track;
    Eigen::MatrixXd sss_waterfall_depth;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > poss;

public:

    sss_map_image_builder(const sss_map_image::BoundsT& bounds, double resolution, int nbr_pings);

    bool empty();

    Eigen::MatrixXd downsample_cols(const Eigen::MatrixXd& M, int new_cols);

    sss_map_image finish();

    void add_waterfall_images(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                              const xtf_sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left);

    void add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                  const xtf_sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

sss_patch_views::ViewsT convert_maps_to_patches(const sss_map_image::ImagesT& map_images, const Eigen::MatrixXd& height_map, double patch_size);

#endif // SSS_MAP_IMAGE_H
