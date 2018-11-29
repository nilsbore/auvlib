#include <bathy_maps/patch_views.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

sss_patch_assembler::sss_patch_assembler(int image_size, double world_size) :
    image_size(image_size), world_size(world_size), is_active_(false)
{
    height_count = Eigen::MatrixXd::Zero(image_size, image_size);
    height_value = Eigen::MatrixXd::Zero(image_size, image_size);

    current_view_count = Eigen::MatrixXd::Zero(image_size, image_size);
    current_view_value = Eigen::MatrixXd::Zero(image_size, image_size);
}

bool sss_patch_assembler::empty()
{
    return patch_views.sss_views.empty(); // current_pos_count == 0;
}

void sss_patch_assembler::activate(const Eigen::Vector3d& new_origin)
{
    origin = new_origin;
    height_count.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);
    height_value.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);
    is_active_ = true;

    patch_views = sss_patch_views();
    patch_views.patch_origin = origin;
    current_pos_value.setZero(); // = Eigen::Vector3d::Zero();
    current_pos_count = 0;
    current_view_count.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);
    current_view_value.setZero(); // = Eigen::MatrixXd::Zero(image_size, image_size);

    nbr_splits = 0;
}

bool sss_patch_assembler::is_active()
{
    return is_active_;
}

void sss_patch_assembler::split()
{
    if (current_pos_count > 0 && current_view_count.sum() > 0) {
        Eigen::MatrixXd current_view(image_size, image_size);
        current_view_count.array() += (current_view_count.array() == 0).cast<double>();
        current_view.array() = current_view_value.array() / current_view_count.array();
        patch_views.sss_views.push_back(current_view);
        patch_views.patch_view_pos.push_back(1./double(current_pos_count)*current_pos_value - origin);
        Eigen::Vector3d direction = 1./double(current_pos_count)*current_pos_value - first_pos;
        patch_views.patch_view_dirs.push_back(1./direction.norm()*direction);

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

sss_patch_views sss_patch_assembler::finish()
{
    split();
    is_active_ = false;
    patch_views.patch_height = Eigen::MatrixXd::Zero(image_size, image_size);
    if (height_count.sum() > 0) {
        patch_views.patch_height.array() = height_value.array() / height_count.array();
    }
    return patch_views;
}

void sss_patch_assembler::add_hits(const Eigen::MatrixXd& hits, const Eigen::Vector3d& pos)
{
    if (hits.rows() == 0) {
        return;
    }

    if (current_pos_count == 0) {
        first_pos = pos;
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
