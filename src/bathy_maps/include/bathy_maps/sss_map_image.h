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

    Eigen::MatrixXd sss_waterfall_image;
    Eigen::MatrixXd sss_waterfall_cross_track;
    Eigen::MatrixXd sss_waterfall_depth;

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

    sss_map_image_builder(const sss_map_image::BoundsT& bounds, double resolution, int nbr_pings) : 
        bounds(bounds), resolution(resolution), waterfall_width(2*nbr_pings), waterfall_counter(0)
    {
        global_origin = Eigen::Vector3d(bounds(0, 0), bounds(0, 1), 0.);

        image_cols = resolution*(bounds(1, 0) - bounds(0, 0));
        image_rows = resolution*(bounds(1, 1) - bounds(0, 1));

        sss_map_image_counts = Eigen::MatrixXd::Zero(image_rows, image_cols);
        sss_map_image_sums = Eigen::MatrixXd::Zero(image_rows, image_cols);

        sss_waterfall_image = Eigen::MatrixXd::Zero(2000, waterfall_width);
        sss_waterfall_cross_track = Eigen::MatrixXd::Zero(2000, waterfall_width);
        sss_waterfall_depth = Eigen::MatrixXd::Zero(2000, waterfall_width);
    }

    bool empty()
    {
        return sss_map_image_counts.sum() == 0;
    }

    /*
    void activate(int nbr_pings)
    {

    }

    bool is_active()
    {

    }

    void split()
    {

    }
    */

    sss_map_image finish()
    {
        sss_map_image map_image;
        map_image.bounds = bounds;
        if (sss_map_image_counts.sum() > 0) {
            sss_map_image_counts.array() += (sss_map_image_counts.array() == 0).cast<double>();
            map_image.sss_map_image.array() = sss_map_image_sums.array() / sss_map_image_counts.array();
        }
        map_image.pos = poss;
        map_image.sss_waterfall_image = sss_waterfall_image.topRows(waterfall_counter);
        map_image.sss_waterfall_cross_track = sss_waterfall_cross_track.topRows(waterfall_counter);
        map_image.sss_waterfall_depth = sss_waterfall_depth.topRows(waterfall_counter);

        return map_image;
    }

    void add_waterfall_images(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                              const xtf_sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left)
    {
        if (waterfall_counter > sss_waterfall_image.rows()) {
            sss_waterfall_image.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
            sss_waterfall_cross_track.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
            sss_waterfall_depth.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        }

        for (int i = 0; i < ping.pings.size(); ++i) {
            int col;
            if (is_left) {
                col = waterfall_width/2 + i;
            }
            else {
                col = waterfall_width/2 - 1 - i;
            }
            sss_waterfall_image(waterfall_counter, col) = double(ping.pings[i])/10000.;
        }

        for (int i = 0; i < hits.rows(); ++i) {
            int ping_ind = hits_inds[i];
            int col;
            if (is_left) {
                col = waterfall_width/2 + ping_ind;
            }
            else {
                col = waterfall_width/2 - 1 - ping_ind;
            }
            sss_waterfall_cross_track(waterfall_counter, col) = (hits.row(i).head<2>() - pos.head<2>().transpose()).norm();
            sss_waterfall_depth(waterfall_counter, col) = hits(i, 2.) - pos(2);
        }

        if (!is_left) {
            ++waterfall_counter;
        }
    }

    void add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                  const xtf_sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left)
    {
        if (hits.rows() == 0) {
            return;
        }

        poss.push_back(pos);

        Eigen::VectorXd intensities = hits.col(3);
        Eigen::MatrixXd points = hits.leftCols<3>();
        // The hits are already compensated to start at bounds.row(0)
        //points.array().rowwise() -= global_origin.array().transpose();
        points.leftCols<2>().array() *= resolution; //*points.leftCols<2>().array();

        std::cout << "Hits rows: " << hits.rows() << std::endl;
        std::cout << "Origin: " << global_origin.transpose() << ", pose: " << pos.transpose() << std::endl;

        int inside_image = 0;
        for (int i = 0; i < points.rows(); ++i) {
            int x = int(points(i, 0));
            int y = int(points(i, 1));
            if (x >= 0 && x < image_cols && y >= 0 && y < image_rows) {
                sss_map_image_sums(y, x) += intensities(i);
                sss_map_image_counts(y, x) += 1.;
                ++inside_image;
            }
        }
        std::cout << "Number inside image: " << inside_image << std::endl;

        add_waterfall_images(hits, hits_inds, ping, pos, is_left);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

sss_patch_views::ViewsT convert_maps_to_patches(const sss_map_image::ImagesT& map_images, const Eigen::MatrixXd& height_map, double patch_size);

#endif // SSS_MAP_IMAGE_H
