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
        //cv::imshow(std::string("Patch")+std::to_string(patch_views.sss_views.size()), img);
        //cv::waitKey(10);
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

void sss_patch_assembler::add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXd& intensities, const Eigen::Vector3d& pos)
{
    if (hits.rows() == 0) {
        return;
    }

    if (current_pos_count == 0) {
        first_pos = pos;
    }

    //Eigen::VectorXd intensities = hits.col(3);
    //Eigen::MatrixXd points = hits.leftCols<3>();
    
    Eigen::MatrixXd points = hits;
    points.array().rowwise() -= origin.array().transpose();
    points.leftCols<2>().array() = double(image_size)/world_size*points.leftCols<2>().array() + double(image_size)/2.;

    std::cout << "Is active?: " << is_active_ << ", nbr pos: " << current_pos_count << std::endl;
    std::cout << "Hits rows: " << points.rows() << std::endl;
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
