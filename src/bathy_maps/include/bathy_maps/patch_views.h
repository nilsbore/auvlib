#ifndef PATCH_VIEWS_H
#define PATCH_VIEWS_H

#include <eigen3/Eigen/Dense>
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

    bool empty();

    void activate(const Eigen::Vector3d& new_origin);

    bool is_active();

    void split();

    sss_patch_views finish();

    void add_hits(const Eigen::MatrixXd& hits, const Eigen::Vector3d& pos);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif // PATCH_VIEWS_H
