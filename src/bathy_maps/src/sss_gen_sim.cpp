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

#include <bathy_maps/sss_gen_sim.h>

#include <igl/readSTL.h>
#include <igl/unproject_onto_mesh.h>
#include <bathy_maps/drape_mesh.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace xtf_data;
using namespace csv_data;

SSSGenSim::SSSGenSim(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                     const xtf_sss_ping::PingsT& pings,
                     const BoundsT& bounds,
                     const csv_asvp_sound_speed::EntriesT& sound_speeds,
                     const Eigen::MatrixXd& height_map)
    : BaseDraper(V1, F1, pings, bounds, sound_speeds),
      bounds(bounds), gen_callback(&default_callback),
      height_map(height_map),
      sss_from_waterfall(false)
{
    viewer.callback_pre_draw = std::bind(&SSSGenSim::callback_pre_draw, this, std::placeholders::_1);
    window_point = Eigen::Vector3d::Zero();
    window_heading = 0.;
    height_map_cv = cv::Mat(height_map.rows(), height_map.cols(), CV_32FC1);
    for (int i = 0; i < height_map.rows(); ++i) {
        for (int j = 0; j < height_map.cols(); ++j) {
            height_map_cv.at<float>(i, j) = height_map(i, j);
        }
    }
    nbr_windows = 256; //(pings[0].port.pings.size() + pings[0].stbd.pings.size())/(2*20);
    cout << "Nbr windows: " << nbr_windows << endl;
    waterfall_image = cv::Mat::zeros(1000, 2*nbr_windows, CV_8UC1);
    gt_waterfall_image = cv::Mat::zeros(1000, 2*nbr_windows, CV_8UC1);
    texture = Eigen::MatrixXd::Zero(20, 9*20);

    waterfall_depth = Eigen::MatrixXd::Zero(32, 2*nbr_windows);
    waterfall_row = 0;
}

Eigen::MatrixXd scale_height_map(const Eigen::MatrixXd& height_map)
{
    Eigen::ArrayXXd height_map_array = height_map.array();
    double minv = height_map.minCoeff();
    height_map_array -= minv*(height_map_array < 0).cast<double>();
    double maxv = height_map_array.maxCoeff();
    height_map_array /= maxv;
    return height_map_array.matrix();
}

void SSSGenSim::generate_sss_window()
{
    cv::Mat height_map_vis_cv(height_map.rows(), height_map.cols(), CV_8UC1);
    Eigen::MatrixXd scaled = scale_height_map(height_map);
    for (int i = 0; i < height_map.rows(); ++i) {
        for (int j = 0; j < height_map.cols(); ++j) {
            height_map_vis_cv.at<uint8_t>(i, j) = uint8_t(255.*scaled(i, j));
        }
    }

    double heading = 180./M_PI*(pings[i].heading_ + sensor_yaw);

    //resolution = (bounds[1, 0] - bounds[0, 0])/1333.
    double resolution = double(height_map_cv.cols)/(bounds(1, 0) - bounds(0, 0));
    Eigen::Vector3d pos = resolution*window_point;

    cv::Size2f cv_image_size = cv::Size2f(20.+5., 9*20.+5.);
    cv::RotatedRect Rect = cv::RotatedRect(cv::Point2f(pos(0), pos(1)), cv_image_size, heading);
    cv::Point2f vertices[4];
    Rect.points(vertices);

    cv::Mat vis_image;
    cv::applyColorMap(height_map_vis_cv, vis_image, cv::COLORMAP_JET);
    for (int i = 0; i < 4; i++) {
        cv::line(vis_image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,0,255), 1);
    }

    cv::Rect brect = Rect.boundingRect();

    bool is_inside = (brect & cv::Rect(0, 0, height_map_cv.cols, height_map_cv.rows)) == brect;
    
    if (!is_inside) {
        cout << "Not inside!" << endl;
        return;
    }

    cv::rectangle(vis_image, brect, cv::Scalar(255,0,0), 1);

    cv::imshow("Aligned rect", vis_image);

    cv::Mat roi = height_map_cv(brect);
    cv::Mat roi_vis = vis_image(brect);

    cv::Point center = cv::Point(roi.cols/2, roi.rows/2);

    cv::Mat rot = cv::getRotationMatrix2D(center, heading, 1.0);

    cv::Mat rotated;
    cv::Mat rotated_vis;
    cv::warpAffine(roi, rotated, rot, roi.size(), cv::INTER_CUBIC);
    cv::warpAffine(roi_vis, rotated_vis, rot, roi.size(), cv::INTER_CUBIC);

    cv::Point rotcenter = cv::Point(rotated.cols/2, rotated.rows/2);
    cv::Mat cropped;
    cv::Mat cropped_vis;
    cv_image_size = cv::Size2f(20., 9*20.);
    cv::getRectSubPix(rotated, cv_image_size, rotcenter, cropped);
    cv::getRectSubPix(rotated_vis, cv_image_size, rotcenter, cropped_vis);

    cv::imshow("Cropped", cropped_vis);
    cv::waitKey(10);

    Eigen::MatrixXd UV = get_UV(V);

    //Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> texture(cropped.cols, cropped.rows);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> Rtext(cropped.cols, cropped.rows);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> Gtext(cropped.cols, cropped.rows);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> Btext(cropped.cols, cropped.rows);

    Eigen::MatrixXd bathy_window(cropped.cols, cropped.rows);
    for (int i = 0; i < cropped.rows; ++i) {
        for (int j = 0; j < cropped.cols; ++j) {
            bathy_window(j, cropped.rows-i-1) = cropped.at<float>(i, j);
        }
    }
    Eigen::MatrixXd generated = gen_callback(bathy_window);

    for (int i = 0; i < generated.rows(); ++i) {
        for (int j = 0; j < generated.cols(); ++j) {
            if (true) {
                Rtext(i, generated.cols()-j-1) = uint8_t(255.*generated(i, j));
                Gtext(i, generated.cols()-j-1) = uint8_t(255.*generated(i, j));
                Btext(i, generated.cols()-j-1) = uint8_t(255.*generated(i, j));
                texture(i, generated.cols()-j-1) = generated(i, j);
            }
            else {
                //texture(j, i) = uint8_t(255.*cropped.at<float>(i, j));
                cv::Point3_<uchar> p = cropped_vis.at<cv::Point3_<uchar> >(i, j);
                Rtext(j, i) = p.z;
                Gtext(j, i) = p.y;
                Btext(j, i) = p.x;
            }
        }
    }

    C.array().rowwise() = Eigen::Vector3d::Ones().array().transpose();
    viewer.data().set_uv(UV);
    viewer.data().show_texture = true;
    //viewer.data().set_texture(texture, texture, texture);
    viewer.data().set_texture(Rtext, Gtext, Btext);

}

pair<Eigen::MatrixXd, Eigen::MatrixXd> SSSGenSim::project()
{
    cout << "Setting new position: " << pings[i].pos_.transpose() << endl;
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Ry*Rcomp;

    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    Eigen::VectorXi hits_left_inds;
    Eigen::VectorXi hits_right_inds;
    Eigen::VectorXd mod_left;
    Eigen::VectorXd mod_right;

    auto start = chrono::high_resolution_clock::now();
    tie(hits_left, hits_right, hits_left_inds, hits_right_inds, mod_left, mod_right) = embree_compute_hits(pings[i].pos_ - offset, R, 1.4*pings[i].port.tilt_angle, pings[i].port.beam_width, V1, F1);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "embree_compute_hits time: " << duration.count() << " microseconds" << endl;

    return make_pair(hits_left, hits_right);
}

Eigen::MatrixXd SSSGenSim::get_UV(const Eigen::MatrixXd& P)
{
    double resolution = double(height_map_cv.cols)/(bounds(1, 0) - bounds(0, 0));
    double heading = window_heading; // pings[i].heading_ + sensor_yaw;
    Eigen::MatrixXd UV = P.leftCols<2>();
    UV.array().rowwise() -= window_point.head<2>().transpose().array();
    Eigen::Matrix2d R = Eigen::AngleAxisd(-heading, Eigen::Vector3d::UnitZ()).matrix().topLeftCorner<2, 2>();
    UV *= R.transpose();
    UV.col(0).array() /= 1./resolution*double(texture.rows());
    UV.col(1).array() /= 1./resolution*double(texture.cols());
    UV.array().rowwise() += 0.5*Eigen::RowVector2d::Ones().array();
    return UV;
}

Eigen::VectorXd SSSGenSim::get_texture_intensities(const Eigen::MatrixXd& P)
{
    Eigen::MatrixXd UV = get_UV(P);
    Eigen::VectorXd intensities = Eigen::VectorXd::Zero(UV.rows());

    for (int i = 0; i < UV.rows(); ++i) {
        if (UV(i, 0) < 1. && UV(i, 0) > 0. && UV(i, 1) < 1. && UV(i, 1) > 0.) {
            int row = int(double(texture.rows())*UV(i, 0));
            int col = int(double(texture.cols())*UV(i, 1));
            intensities(i) = texture(row, col);
        }
        else {
            cout << "Texture coordinates not correct: " << UV(i, 0) << ", " << UV(i, 1) << endl;
        }
    }

    return intensities;
}

Eigen::VectorXd SSSGenSim::compute_times(const Eigen::MatrixXd& P)
{
    Eigen::Vector3d pos = pings[i].pos_ - offset;
    double sound_vel = sound_speeds[0].vels.head(sound_speeds[0].vels.rows()-1).mean();
    Eigen::VectorXd times = 2.*(P.rowwise() - pos.transpose()).rowwise().norm()/sound_vel;
    return times;
}

Eigen::VectorXd SSSGenSim::compute_time_windows(const Eigen::VectorXd& times, const Eigen::VectorXd& intensities, const xtf_data::xtf_sss_ping_side& ping)
{
    double ping_step = ping.time_duration / double(nbr_windows);
    Eigen::VectorXd time_windows = Eigen::VectorXd::Zero(nbr_windows);
    Eigen::VectorXd time_counts = Eigen::VectorXd::Zero(nbr_windows);
    for (int i = 0; i < times.rows(); ++i) {
        int index = int(times(i)/ping_step);
        if (index < nbr_windows) {
            time_windows(index) += intensities(i);
            time_counts(index) += 1.;
        }
    }
    time_windows.array() /= time_counts.array();
    return time_windows;
}

Eigen::VectorXd SSSGenSim::compute_depth_windows(const Eigen::VectorXd& times, const Eigen::MatrixXd& hits, const xtf_data::xtf_sss_ping_side& ping)
{
    double ping_step = ping.time_duration / double(nbr_windows);
    Eigen::VectorXd depth_windows = Eigen::VectorXd::Zero(nbr_windows);
    Eigen::VectorXd depth_counts = Eigen::VectorXd::Zero(nbr_windows);
    for (int i = 0; i < times.rows(); ++i) {
        int index = int(times(i)/ping_step);
        if (index < nbr_windows) {
            depth_windows(index) += hits(i, 2);
            depth_counts(index) += 1.;
        }
    }
    depth_windows.array() /= depth_counts.array();
    return depth_windows;
}

/*
Eigen::VectorXd SSSGenSim::match_intensities(const Eigen::VectorXd& times, const xtf_data::xtf_sss_ping_side& ping)
{
    double ping_step = ping.time_duration / double(nbr_windows);
    Eigen::VectorXd time_windows = Eigen::VectorXd::Zero(nbr_windows);
    Eigen::VectorXd time_counts = Eigen::VectorXd::Zero(nbr_windows);
    for (int i = 0; i < times.rows(); ++i) {
        int index = int(times(i)/ping_step);
        if (index < nbr_windows) {
            time_windows(index) += intensities(i);
            time_counts(index) += 1.;
        }
    }
    time_windows.array() /= time_counts.array();
    return time_windows;
}
*/

void SSSGenSim::visualize_rays(const Eigen::MatrixXd& hits_left, const Eigen::MatrixXd& hits_right)
{
    Eigen::MatrixXi E;
    Eigen::MatrixXd P(hits_left.rows(), 3);
    P.rowwise() = (pings[i].pos_ - offset).transpose();
    viewer.data().set_edges(P, E, Eigen::RowVector3d(1., 0., 0.));
    viewer.data().add_edges(P, hits_left, Eigen::RowVector3d(1., 0., 0.));
    P = Eigen::MatrixXd(hits_right.rows(), 3);
    P.rowwise() = (pings[i].pos_ - offset).transpose();
    viewer.data().add_edges(P, hits_right, Eigen::RowVector3d(0., 1., 0.));
}

void SSSGenSim::visualize_vehicle()
{
    if (V2.rows() == 0) {
        return;
    }
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Ry*Rcomp;

    V.bottomRows(V2.rows()) = V2;
    V.bottomRows(V2.rows()) *= R.transpose();
    V.bottomRows(V2.rows()).array().rowwise() += (pings[i].pos_ - offset).transpose().array();
    viewer.data().set_vertices(V);
}

void SSSGenSim::construct_gt_waterfall()
{
    cv::Mat shifted = cv::Mat::zeros(waterfall_image.rows, waterfall_image.cols, waterfall_image.type());
    gt_waterfall_image(cv::Rect(0, 0, waterfall_image.cols, waterfall_image.rows-1)).copyTo(shifted(cv::Rect(0, 1, shifted.cols, shifted.rows-1)));
    shifted.copyTo(gt_waterfall_image);

    double ping_step = double(pings[i].port.pings.size())/double(nbr_windows);
    for (int j = 0; j < pings[i].port.pings.size(); ++j) {
        double left_intensity = double(pings[i].port.pings[j])/(10000.);
        double right_intensity = double(pings[i].stbd.pings[j])/(10000.);

        int left_index = nbr_windows-int(double(j)/ping_step)-1;
        int right_index = nbr_windows+int(double(j)/ping_step);

        if (left_index > 0) {
            gt_waterfall_image.at<uint8_t>(0, left_index) = uint8_t(255.*left_intensity);
        }
        if (right_index < 2*nbr_windows) {
            gt_waterfall_image.at<uint8_t>(0, right_index) = uint8_t(255.*right_intensity);
        }

    }
}

bool SSSGenSim::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    if (i >= pings.size()) {
        return false;
    }

    if (sss_from_waterfall) {
        if (waterfall_row == 32) {
            Eigen::MatrixXd generated = gen_callback(waterfall_depth);
            for (int row = 0; row < 32; ++row) {
                for (int col = 0; col < generated.cols(); ++col) {
                    waterfall_image.at<uint8_t>(row, col) = uint8_t(255.*generated(row, col));
                }
            }
            waterfall_row = 0;
        }
    }
    else {
        if ((window_point - (pings[i].pos_ - offset)).norm() > 4.) {
            window_point = pings[i].pos_ - offset;
            window_heading = pings[i].heading_ + sensor_yaw;
            generate_sss_window();
        }
    }

    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    tie(hits_left, hits_right) = project();

    Eigen::VectorXd times_left = compute_times(hits_left);
    Eigen::VectorXd times_right = compute_times(hits_right);

    cv::Mat shifted = cv::Mat::zeros(waterfall_image.rows, waterfall_image.cols, waterfall_image.type());
    waterfall_image(cv::Rect(0, 0, waterfall_image.cols, waterfall_image.rows-1)).copyTo(shifted(cv::Rect(0, 1, shifted.cols, shifted.rows-1)));
    shifted.copyTo(waterfall_image);

    if (sss_from_waterfall) {

        Eigen::VectorXd depth_windows_left = compute_depth_windows(times_left, hits_left, pings[i].port);
        Eigen::VectorXd depth_windows_right = compute_depth_windows(times_right, hits_right, pings[i].stbd);

        Eigen::VectorXd depth_windows(depth_windows_left.rows() + depth_windows_right.rows());
        depth_windows.tail(depth_windows_right.rows()) = depth_windows_right;
        depth_windows.head(depth_windows_left.rows()) = depth_windows_left.reverse();

        waterfall_depth.row(32-waterfall_row-1) = depth_windows.transpose();
        ++waterfall_row;

    }
    else {

        Eigen::VectorXd intensities_left = get_texture_intensities(hits_left);
        Eigen::VectorXd intensities_right = get_texture_intensities(hits_right);

        Eigen::VectorXd time_windows_left = compute_time_windows(times_left, intensities_left, pings[i].port);
        Eigen::VectorXd time_windows_right = compute_time_windows(times_right, intensities_right, pings[i].stbd);

        Eigen::VectorXd time_windows(time_windows_left.rows() + time_windows_right.rows());
        time_windows.tail(time_windows_right.rows()) = time_windows_right;
        time_windows.head(time_windows_left.rows()) = time_windows_left.reverse();

        for (int i = 0; i < std::min(time_windows.rows(), long(waterfall_image.cols)); ++i) {
            waterfall_image.at<uint8_t>(0, i) = uint8_t(255.*time_windows(i));
        }

    }

    construct_gt_waterfall();

    cv::imshow("Waterfall image", waterfall_image);

    cv::imshow("Ground truth waterfall image", gt_waterfall_image);

    cv::waitKey(10);

    if (i % 10 == 0) {
        visualize_rays(hits_left, hits_right);
        visualize_vehicle();
    }

    ++i;

    return false;
}
