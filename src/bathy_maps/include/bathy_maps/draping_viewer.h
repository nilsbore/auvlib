#ifndef DRAPING_VIEWER_H
#define DRAPING_VIEWER_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>
#include <igl/unproject_onto_mesh.h>

#include <eigen3/Eigen/Dense>
#include <data_tools/xtf_data.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

struct sss_patch_views {

    using ViewsT = std::vector<sss_patch_views, Eigen::aligned_allocator<sss_patch_views> >;

    Eigen::MatrixXd patch_height;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > sss_views;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > patch_view_pos;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(patch_height), CEREAL_NVP(sss_views), CEREAL_NVP(patch_view_pos));
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
    int current_pos_count;
    Eigen::MatrixXd current_view_value;
    Eigen::MatrixXd current_view_count;

    Eigen::MatrixXd height_value;
    Eigen::MatrixXd height_count;

    sss_patch_views patch_views;

    // DEBUG
    int nbr_splits;

public:

    sss_patch_assembler(int image_size=30, double world_size=8.) : image_size(image_size), world_size(world_size), is_active_(false)
    {
        height_count = Eigen::MatrixXd::Zero(image_size, image_size);
        height_value = Eigen::MatrixXd::Zero(image_size, image_size);

        current_view_count = Eigen::MatrixXd::Zero(image_size, image_size);
        current_view_value = Eigen::MatrixXd::Zero(image_size, image_size);
    }

    bool empty()
    {
        return patch_views.sss_views.empty(); // current_pos_count == 0;
    }

    void activate(const Eigen::Vector3d& new_origin)
    {
        origin = new_origin;
        height_count.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);
        height_value.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);
        is_active_ = true;

        patch_views = sss_patch_views();
        current_pos_value.setZero(); // = Eigen::Vector3d::Zero();
        current_pos_count = 0;
        current_view_count.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);
        current_view_value.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);

        nbr_splits = 0;
    }

    bool is_active()
    {
        return is_active_;
    }

    void split()
    {
        if (current_pos_count > 0 && current_view_count.sum() > 0) {
            Eigen::MatrixXd current_view(image_size, image_size);
            current_view_count.array() += (current_view_count.array() == 0).cast<double>();
            current_view.array() = current_view_value.array() / current_view_count.array();
            patch_views.sss_views.push_back(current_view);
            patch_views.patch_view_pos.push_back(1./double(current_pos_count)*current_pos_value);

            cv::Mat img = cv::Mat(image_size, image_size, CV_8UC1, cv::Scalar(0));
            for (int row = 0; row < image_size; ++row) {
                for (int col = 0; col < image_size; ++col) {
                    //double value = 255.*current_view(row, col); //patch_views.sss_views.back()(row, col);
                    double value = 255.*patch_views.sss_views.back()(row, col);
                    std::cout << row << ", " << col << ": " << value << std::endl;
                    if (value < 256 && value > 0) {
                        img.at<uchar>(row, col) = uchar(value);
                    }
                }
            }
            cv::imshow(std::string("Patch")+std::to_string(patch_views.sss_views.size()), img);
            cv::waitKey(10);
        }

        current_pos_value.setZero(); // = Eigen::Vector3d::Zero();
        current_pos_count = 0;
        current_view_count.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);
        current_view_value.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);

        nbr_splits += 1;
    }

    sss_patch_views finish()
    {
        split();
        is_active_ = false;
        patch_views.patch_height = Eigen::MatrixXd::Zero(image_size, image_size);
        if (height_count.sum() > 0) {
            patch_views.patch_height.array() = height_value.array() / height_count.array();
        }
        return patch_views;
    }

    void add_hits(const Eigen::MatrixXd& hits, const Eigen::Vector3d& pos)
    {
        if (hits.rows() == 0) {
            return;
        }

        Eigen::VectorXd intensities = hits.col(3);
        Eigen::MatrixXd points = hits.leftCols<3>();
        points.array().rowwise() -= origin.array().transpose();
        points.leftCols<2>().array() = double(image_size)/world_size*points.leftCols<2>().array() + double(image_size)/2.;

        std::cout << "Is active?: " << is_active_ << ", nbr pos: " << current_pos_count << std::endl;
        std::cout << "Hits rows: " << hits.rows() << std::endl;
        std::cout << "Origin: " << origin.transpose() << ", pose: " << pos.transpose() << std::endl;

        current_pos_value.array() += pos.array();
        current_pos_count += 1;

        int inside_image = 0;
        for (int i = 0; i < points.rows(); ++i) {
            int x = int(points(i, 0));
            int y = int(points(i, 1));
            if (x >= 0 && x < image_size && y >= 0 && y < image_size) {
                current_view_value(y, x) += intensities(i);
                current_view_count(y, x) += 1.;

                height_value(y, x) += points(i, 2);
                height_count(y, x) += 1.;
                ++inside_image;
            }
        }
        std::cout << "Number inside image: " << inside_image << std::endl;
        std::cout << "Is active?: " << is_active_ << ", nbr pos: " << current_pos_count << std::endl;
        std::cout << "number splits: " << nbr_splits << ", nbr added: " << patch_views.sss_views.size() << std::endl;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

struct survey_viewer {
public:

    using BoundsT = Eigen::Matrix2d;

private:

    igl::opengl::glfw::Viewer viewer;
    const xtf_sss_ping::PingsT& pings;
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

    sss_patch_assembler patch_assembler;
    sss_patch_views::ViewsT patch_views;
    Eigen::VectorXi is_active;

public:

    survey_viewer(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
        const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
        const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset);

    void launch();
    void project_sss();
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
    bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int);
    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods);
    sss_patch_views::ViewsT get_patch_views();
};

sss_patch_views::ViewsT overlay_sss(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                    const survey_viewer::BoundsT& bounds, const xtf_sss_ping::PingsT& pings);

#endif // DRAPING_VIEWER_H
