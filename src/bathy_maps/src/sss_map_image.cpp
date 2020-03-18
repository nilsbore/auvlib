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

#include <bathy_maps/sss_map_image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

sss_map_image_builder::sss_map_image_builder(const sss_map_image::BoundsT& bounds, double resolution, int nbr_pings) : 
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
    sss_waterfall_model = Eigen::MatrixXd::Zero(2000, waterfall_width);
}

pair<int, int> sss_map_image_builder::get_map_image_shape()
{
    return make_pair(image_rows, image_cols);
}

size_t sss_map_image_builder::get_waterfall_bins()
{
    return waterfall_width / 2;
}

bool sss_map_image_builder::empty()
{
    return sss_map_image_counts.sum() == 0;
}

Eigen::MatrixXd sss_map_image_builder::downsample_cols(const Eigen::MatrixXd& M, int new_cols)
{
    if (new_cols == M.cols()) {
        return M;
    }

    double factor = double(new_cols)/double(M.cols());
    Eigen::ArrayXXd sums = Eigen::MatrixXd::Zero(M.rows(), new_cols);
    Eigen::ArrayXXd counts = Eigen::MatrixXd::Zero(M.rows(), new_cols);

    int ind;
    for (int i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < M.cols(); ++j) {
            ind = int(factor*j);
            if (M(i, j) != 0) {
                sums(i, ind) += M(i, j);
                counts(i, ind) += 1.;
            }
        }
    }

    counts += (counts == 0).cast<double>();
    sums /= counts;

    return sums.matrix();
}

sss_map_image sss_map_image_builder::finish()
{
    sss_map_image map_image;
    map_image.bounds = bounds;
    if (sss_map_image_counts.sum() > 0) {
        sss_map_image_counts.array() += (sss_map_image_counts.array() == 0).cast<double>();
        map_image.sss_map_image_.array() = sss_map_image_sums.array() / sss_map_image_counts.array();
    }
    map_image.sss_ping_duration = sss_ping_duration;
    map_image.pos = poss;
    if (waterfall_width == 512) {
        map_image.sss_waterfall_image = sss_waterfall_image.topRows(waterfall_counter).cast<float>();
        map_image.sss_waterfall_depth = sss_waterfall_depth.topRows(waterfall_counter).cast<float>();
        map_image.sss_waterfall_model = sss_waterfall_model.topRows(waterfall_counter).cast<float>();
    }
    else {
        map_image.sss_waterfall_image = downsample_cols(sss_waterfall_image.topRows(waterfall_counter), 512).cast<float>();
        //map_image.sss_waterfall_cross_track = sss_waterfall_cross_track.topRows(waterfall_counter);
        map_image.sss_waterfall_depth = downsample_cols(sss_waterfall_depth.topRows(waterfall_counter), 512).cast<float>();
        map_image.sss_waterfall_model = downsample_cols(sss_waterfall_model.topRows(waterfall_counter), 512).cast<float>();
    }

    return map_image;
}

void sss_map_image_builder::add_waterfall_images(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                                                 const std_data::sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left)
{
    if (waterfall_counter >= sss_waterfall_image.rows()) {
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

void sss_map_image_builder::add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                                     const std_data::sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left)
{
    if (hits.rows() == 0) {
        return;
    }

    sss_ping_duration = ping.time_duration;
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

void sss_map_image_builder::add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                                     const Eigen::VectorXd& intensities,
                                     const Eigen::VectorXd& sss_depths, const Eigen::VectorXd& sss_model,
                                     const std_data::sss_ping_side& ping, const Eigen::Vector3d& pos,
                                     const Eigen::Vector3d& rpy, bool is_left)
{
    /*
    poss.push_back(pos);

    // this might be the culprit
    if (hits.rows() == 0) {
        if (!is_left) {
            ++waterfall_counter;
        }
        return;
    }
    */

    // this might be the culprit
    if (hits.rows() == 0) {
        return;
    }

    sss_ping_duration = ping.time_duration;
    poss.push_back(pos);

    Eigen::MatrixXd points = hits;
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

    if (waterfall_counter >= sss_waterfall_image.rows()) {
        std::cout << __FILE__ << ", " << __LINE__ << std::endl;
        sss_waterfall_image.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        std::cout << __FILE__ << ", " << __LINE__ << std::endl;
        sss_waterfall_cross_track.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        std::cout << __FILE__ << ", " << __LINE__ << std::endl;
        sss_waterfall_depth.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        std::cout << __FILE__ << ", " << __LINE__ << std::endl;
        sss_waterfall_model.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
    }

    std::cout << __FILE__ << ", " << __LINE__ << std::endl;

    if (waterfall_width == 512) {
        double ping_step = double(ping.pings.size()) / double(waterfall_width/2);
        Eigen::ArrayXd value_windows = Eigen::VectorXd::Zero(waterfall_width/2);
        Eigen::ArrayXd value_counts = Eigen::ArrayXd::Zero(waterfall_width/2);
        for (int i = 0; i < ping.pings.size(); ++i) {
            int col = int(double(i)/ping_step);
            value_windows(col) += double(ping.pings[i])/10000.;
            value_counts(col) += 1.;
        }
        value_counts += (value_counts == 0).cast<double>();
        if (is_left) {
            sss_waterfall_image.block(waterfall_counter, waterfall_width/2, 1, waterfall_width/2) = (value_windows / value_counts).transpose();
        }
        else {
            sss_waterfall_image.block(waterfall_counter, 0, 1, waterfall_width/2) = (value_windows / value_counts).reverse().transpose();
        }
    }
    else {
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
    }
    std::cout << __FILE__ << ", " << __LINE__ << std::endl;

    if (is_left) {
        sss_waterfall_depth.block(waterfall_counter, waterfall_width/2, 1, waterfall_width/2) = sss_depths.transpose();
        sss_waterfall_model.block(waterfall_counter, waterfall_width/2, 1, waterfall_width/2) = sss_model.transpose();
    }
    else {
        sss_waterfall_depth.block(waterfall_counter, 0, 1, waterfall_width/2) = sss_depths.reverse().transpose();
        sss_waterfall_model.block(waterfall_counter, 0, 1, waterfall_width/2) = sss_model.reverse().transpose();
    }
    std::cout << __FILE__ << ", " << __LINE__ << std::endl;

    if (!is_left) {
        ++waterfall_counter;
    }
}

sss_patch_views::ViewsT convert_maps_to_patches(const sss_map_image::ImagesT& map_images, const Eigen::MatrixXd& height_map, double patch_size)
{
    sss_patch_views::ViewsT patches;

    sss_map_image::BoundsT bounds = map_images[0].bounds;
    int image_rows = map_images[0].sss_map_image_.rows();
    int image_cols = map_images[0].sss_map_image_.cols();

    double resolution = double(image_cols)/(bounds(1, 0) - bounds(0, 0));

    int image_size = patch_size*resolution;

    cout << "Got image size: " << image_size << endl;

    int nbr_patches_x = image_cols / image_size - 1;
    int nbr_patches_y = image_rows / image_size - 1;
    //double patch_area = double(image_size*image_size);

    cout << "Number patches x: " << nbr_patches_x << endl;
    cout << "Number patches y: " << nbr_patches_y << endl;

    cout << "Number rows: " << image_rows << endl;
    cout << "Number cols: " << image_cols << endl;

    for (int i = 0; i < nbr_patches_y; ++i) {
        for (int j = 0; j < nbr_patches_x; ++j) {
            sss_patch_views patch_views;
            patch_views.patch_size = patch_size;
            double x = double(j*image_size + image_size/2)/resolution;
            double y = double(i*image_size + image_size/2)/resolution;
            Eigen::Vector3d origin = Eigen::Vector3d(x, y, 0.);
            patch_views.patch_origin = origin + Eigen::Vector3d(bounds(0, 0), bounds(0, 1), 0.); // find closes to origin in pos
            //patch_views.patch_height = Eigen::MatrixXd::Zero(image_size, image_size);
            patch_views.patch_height = height_map.block(i*image_size, j*image_size, image_size, image_size);
            for (int n = 0; n < map_images.size(); ++n) {
                //cout << "current x start: " << j*image_size << " out of " << map_images[n].sss_map_image_.cols() << endl;
                //cout << "current y start: " << i*image_size << " out of " << map_images[n].sss_map_image_.rows() << endl;
                Eigen::MatrixXd view = map_images[n].sss_map_image_.block(i*image_size, j*image_size, image_size, image_size);
                //cout << "view mean: " << view.mean() << endl;
                double fraction_zeros = (view.array() == 0).cast<double>().mean(); // / patch_area;
                if (fraction_zeros < 1.) {
                    cout << "Mean: " << fraction_zeros << endl;
                }
                if (std::isinf(view.mean()) || fraction_zeros > 0.2) {
                    continue;
                }

                cout << "Accepted mean: " << fraction_zeros << endl;

                patch_views.sss_views.push_back(view);
                auto iter = std::min_element(map_images[n].pos.begin(), map_images[n].pos.end(), [&origin](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
                    return (p1-origin).squaredNorm() < (p2-origin).squaredNorm();
                });
                int ind = std::distance(map_images[n].pos.begin(), iter);
                //cout << "Origin: " << origin.transpose() << endl;
                //cout << "Got ind: " << ind << " out of " << map_images[n].pos.size() << endl;
                patch_views.patch_view_pos.push_back(*iter-origin);
                ind = std::min(int(map_images[n].pos.size())-3, ind);
                Eigen::Vector3d dir = map_images[n].pos[ind+2]-map_images[n].pos[ind];
                //cout << "p1: " << map_images[n].pos[ind+2].transpose() << endl;
                //cout << "p2: " << map_images[n].pos[ind].transpose() << endl;
                //cout << "Dir: " << dir.transpose() << endl;
                dir.normalize();
                patch_views.patch_view_dirs.push_back(dir);
            }
            if (!patch_views.sss_views.empty()) {
                patches.push_back(patch_views);
            }
        }
    }

    return patches;
}

sss_patch_views::ViewsT get_oriented_patches(const cv::Mat& image, const sss_map_image& map_image, double patch_size, double resolution, bool visualize=false, bool normalize_colors=false)
{
    int image_size = patch_size*resolution;

    //cv::Mat vis_image = image.clone();
    double max_value = 1.; double min_value = 0.;
    cv::Mat vis_image = image.clone();
    if (normalize_colors) {
        cv::minMaxLoc(vis_image, &min_value, &max_value);
        vis_image -= min_value;
    }
    vis_image.convertTo(vis_image, CV_8U, 255.0/(max_value-min_value));
    cv::cvtColor(vis_image, vis_image, cv::COLOR_GRAY2BGR);

    vector<vector<cv::Mat> > cv_patches;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > origins;

    Eigen::Vector3d last_pos = map_image.pos[0];
    for (int i = 0; i < map_image.pos.size(); ++i) {
        if ((last_pos - map_image.pos[i]).norm() < patch_size) {
            continue;
        }
        Eigen::Vector3d center_pos = .5*(last_pos + map_image.pos[i]);
        Eigen::Vector3d forward_dir = last_pos - map_image.pos[i];
        forward_dir.normalize();
        double forward_angle = 180./M_PI*atan2(forward_dir(0), -forward_dir(1));

        Eigen::Vector3d right_dir = Eigen::Vector3d(forward_dir(1), -forward_dir(0), 0.);
        right_dir.normalize();

        cv_patches.push_back(vector<cv::Mat>());
        origins.push_back(center_pos);
        for (int j = 1; j <= 4; ++j) {
            Eigen::Vector3d right_pos = resolution*(center_pos + double(j)*patch_size*right_dir);
            Eigen::Vector3d left_pos = resolution*(center_pos - double(j)*patch_size*right_dir);

            //cv::polylines(Mat& img, const Point** pts, const int* npts, 1, true, cv::Scalar(255,0,0), 1); //, int lineType=8, int shift=0)
            cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(right_pos(0), right_pos(1)), cv::Size2f(image_size+2, image_size+2), forward_angle);
            cv::RotatedRect lRect = cv::RotatedRect(cv::Point2f(left_pos(0), left_pos(1)), cv::Size2f(image_size+2, image_size+2), forward_angle);
            cv::Point2f lvertices[4];
            cv::Point2f rvertices[4];
            lRect.points(lvertices);
            rRect.points(rvertices);
            for (int i = 0; i < 4; i++) {
                cv::line(vis_image, rvertices[i], rvertices[(i+1)%4], cv::Scalar(0,0,255), 1);
                cv::line(vis_image, lvertices[i], lvertices[(i+1)%4], cv::Scalar(0,255,0), 1);
            }
            cv::Rect rbrect = rRect.boundingRect();
            cv::Rect lbrect = lRect.boundingRect();

            bool ris_inside = (rbrect & cv::Rect(0, 0, image.cols, image.rows)) == rbrect;
            bool lis_inside = (lbrect & cv::Rect(0, 0, image.cols, image.rows)) == lbrect;
            
            cv::Size2f cv_image_size = cv::Size2f(image_size, image_size);
            if (!ris_inside || !lis_inside) {
                //cv_patches.back().push_back(cv::Mat::zeros(cv_image_size, CV_8UC3));
                //cv_patches.back().push_back(cv::Mat::zeros(cv_image_size, CV_8UC3));
                cv_patches.back().push_back(cv::Mat::zeros(cv_image_size, CV_32FC1));
                cv_patches.back().push_back(cv::Mat::zeros(cv_image_size, CV_32FC1));
                continue;
            }

            cv::rectangle(vis_image, rbrect, cv::Scalar(255,0,0), 1);
            cv::rectangle(vis_image, lbrect, cv::Scalar(255,0,0), 1);

            cv::Mat rroi = image(rbrect);
            cv::Mat lroi = image(lbrect);

            cv::Point rcenter = cv::Point(rroi.rows/2, rroi.cols/2);
            cv::Point lcenter = cv::Point(lroi.rows/2, lroi.cols/2);

            cv::Mat rrot = cv::getRotationMatrix2D(rcenter, forward_angle, 1.0);
            cv::Mat lrot = cv::getRotationMatrix2D(lcenter, forward_angle, 1.0);

            cv::Mat rrotated;
            cv::Mat lrotated;
            cv::warpAffine(rroi, rrotated, rrot, rroi.size(), cv::INTER_CUBIC);
            cv::warpAffine(lroi, lrotated, lrot, lroi.size(), cv::INTER_CUBIC);

            cv::Point rrotcenter = cv::Point(rrotated.rows/2, rrotated.cols/2);
            cv::Point lrotcenter = cv::Point(lrotated.rows/2, lrotated.cols/2);
            cv::Mat rcropped, lcropped;
            cv::getRectSubPix(rrotated, cv_image_size, rrotcenter, rcropped);
            cv::getRectSubPix(lrotated, cv_image_size, lrotcenter, lcropped);

            cv_patches.back().push_back(rcropped);
            cv_patches.back().push_back(lcropped);
        }

        last_pos = map_image.pos[i];
    }

    if (visualize) {
        cv::imshow("My image", vis_image);
    }

    //cv::Mat wf(cv_patches.size()*image_size, 2*4*image_size, CV_8UC3);
    cv::Mat wf(cv_patches.size()*image_size, 2*4*image_size, CV_32FC1);

    //vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > patches;
    //vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > poss;
    sss_patch_views::ViewsT patches;

    for (int i = 0; i < cv_patches.size(); ++i) {
        for (int j = 0; j < cv_patches[i].size(); ++j) {
            int ind = j / 2;
            cv::Mat patch = cv_patches[i][j].clone();
            Eigen::MatrixXd eigen_patch(image_size, image_size);
            for (int row = 0; row < image_size; ++row) {
                for (int col = 0; col < image_size; ++col) {
                    if (j % 2 == 0) {
                        eigen_patch(row, col) = patch.at<float>(row, image_size-1-col);
                    }
                    else {
                        eigen_patch(row, col) = patch.at<float>(row, col);
                    }
                }
            }
            //poss.push_back(Eigen::Vector3d(-(0.5+double(ind)*patch_size), 0., 0.));
            //patches.push_back(eigen_patch);
            sss_patch_views view;
            view.sss_views.push_back(eigen_patch);
            view.patch_view_pos.push_back(Eigen::Vector3d(-double(ind+1)*patch_size, 0., 0.));
            if (j % 2 == 0) {
                view.patch_view_dirs.push_back(Eigen::Vector3d(0., 1., 0.));
            }
            else {
                view.patch_view_dirs.push_back(Eigen::Vector3d(0., -1., 0.));
            }
            view.patch_origin = origins[i];
            view.patch_size = patch_size;
            patches.push_back(view);

            if (normalize_colors) {
                patch -= min_value;
                patch *= 1./(max_value - min_value);
            }
            if (j % 2 == 0) {
                patch.copyTo(wf(cv::Rect(0*image_size+(3-ind)*image_size, i*image_size, image_size, image_size)));
            }
            else {
                patch.copyTo(wf(cv::Rect(4*image_size+ind*image_size, i*image_size, image_size, image_size)));

            }
        }
    }

    if (visualize) {
        cv::imshow("waterfall", wf);
        cv::waitKey(0); //200);
    }

    return patches; //make_pair(patches, poss);
}

sss_patch_views::ViewsT convert_maps_to_single_angle_patches(const sss_map_image::ImagesT& map_images, const Eigen::MatrixXd& height_map, double patch_size)
{
    sss_patch_views::ViewsT patches;

    sss_map_image::BoundsT bounds = map_images[0].bounds;
    int image_cols = map_images[0].sss_map_image_.cols();
    double resolution = double(image_cols)/(bounds(1, 0) - bounds(0, 0));

    int image_size = patch_size*resolution;

    cout << "Got image size: " << image_size << endl;

    for (int n = 0; n < map_images.size(); ++n) {

        cv::Mat sss_image(map_images[n].sss_map_image_.rows(), map_images[n].sss_map_image_.rows(), CV_32FC1);
        cv::Mat map_image(map_images[n].sss_map_image_.rows(), map_images[n].sss_map_image_.rows(), CV_32FC1);
        for (int i = 0; i < sss_image.rows; ++i) {
            for (int j = 0; j < sss_image.cols; ++j) {
                sss_image.at<float>(i, j) = map_images[n].sss_map_image_(i, j);
                map_image.at<float>(i, j) = height_map(i, j);
            }
        }

        bool visualize = true;
        sss_patch_views::ViewsT sss_patches = get_oriented_patches(sss_image, map_images[n], patch_size, resolution, visualize);
        sss_patch_views::ViewsT map_patches = get_oriented_patches(map_image, map_images[n], patch_size, resolution, visualize, true);

        for (int i = 0; i < sss_patches.size(); ++i) {
            sss_patches[i].patch_height = map_patches[i].sss_views[0];
        }
        patches.insert(patches.end(), sss_patches.begin(), sss_patches.end());

    }

    return patches;
}
