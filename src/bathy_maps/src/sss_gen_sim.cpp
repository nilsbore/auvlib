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
//#include <bathy_maps/drape_mesh.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace csv_data;

SSSGenSim::SSSGenSim(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                     const std_data::sss_ping::PingsT& pings,
                     const BoundsT& bounds,
                     const csv_asvp_sound_speed::EntriesT& sound_speeds,
                     const Eigen::MatrixXd& height_map)
    : ViewDraper(V1, F1, pings, bounds, sound_speeds), //bounds(bounds),
      gen_callback(&default_callback),
      height_map(height_map),
      sss_from_waterfall(false),
      sss_from_bathy(false)
{
    resample_window_height = 50; //32;
    full_window_height = 64;

    left_row_mean = 0.;
    right_row_mean = 0.;

    viewer.callback_pre_draw = std::bind(&SSSGenSim::callback_pre_draw, this, std::placeholders::_1);
    viewer.callback_key_pressed = std::bind(&SSSGenSim::callback_key_pressed, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
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
    model_waterfall_image = cv::Mat::zeros(1000, 2*nbr_windows, CV_8UC1);
    texture = Eigen::MatrixXd::Zero(20, 9*20);

    waterfall_depth = Eigen::MatrixXd::Zero(full_window_height, 2*nbr_windows);
    waterfall_model = Eigen::MatrixXd::Zero(full_window_height, 2*nbr_windows);
    waterfall_row = 0;
}

bool SSSGenSim::callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods)
{
    switch (key) {
    case 'n':
        while (i < pings.size() && !pings[i].first_in_file_) {
            ++i;
        }
        return true;
    default:
        return false;
    }
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
    // TODO: the current function is not used atm, but needs this
    // function that does not exist on some opencv versions
    //cv::applyColorMap(height_map_vis_cv, vis_image, cv::COLORMAP_JET);
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

void SSSGenSim::construct_gt_waterfall()
{
    cv::Mat shifted = cv::Mat::zeros(waterfall_image.rows, waterfall_image.cols, waterfall_image.type());
    gt_waterfall_image(cv::Rect(0, 0, waterfall_image.cols, waterfall_image.rows-1)).copyTo(shifted(cv::Rect(0, 1, shifted.cols, shifted.rows-1)));
    shifted.copyTo(gt_waterfall_image);

    Eigen::ArrayXd values = Eigen::ArrayXd::Zero(2*nbr_windows);
    Eigen::ArrayXd value_counts = Eigen::ArrayXd::Zero(2*nbr_windows);

    double ping_step = double(pings[i].port.pings.size())/double(nbr_windows);
    for (int j = 0; j < pings[i].port.pings.size(); ++j) {
        double left_intensity = double(pings[i].port.pings[j])/(10000.);
        double right_intensity = double(pings[i].stbd.pings[j])/(10000.);

        int left_index = nbr_windows-int(double(j)/ping_step)-1;
        int right_index = nbr_windows+int(double(j)/ping_step);

        if (left_index > 0) {
            values(left_index) += left_intensity;
            value_counts(left_index) += 1.;
            //gt_waterfall_image.at<uint8_t>(0, left_index) = uint8_t(255.*left_intensity);
        }
        if (right_index < 2*nbr_windows) {
            values(right_index) += right_intensity;
            value_counts(right_index) += 1.;
            //gt_waterfall_image.at<uint8_t>(0, right_index) = uint8_t(255.*right_intensity);
        }

    }
    value_counts += (value_counts == 0).cast<double>();
    values /= value_counts;

    for (int j = 0; j < 2*nbr_windows; ++j) {
        // TODO: maybe make the intensity multiplication factor a parameter
        //gt_waterfall_image.at<uint8_t>(0, j) = uint8_t(255.*std::min(std::max(fabs(values(j)), 0.), 1.));
        gt_waterfall_image.at<uint8_t>(0, j) = uint8_t(255.*std::min(std::max(fabs(2.*values(j)), 0.), 1.));
    }
}

void SSSGenSim::construct_model_waterfall(const Eigen::MatrixXd& hits_left, const Eigen::MatrixXd& hits_right,
                                          const Eigen::MatrixXd& normals_left, const Eigen::MatrixXd& normals_right,
                                          const Eigen::VectorXd& times_left, const Eigen::VectorXd& times_right)
{
    cv::Mat shifted = cv::Mat::zeros(waterfall_image.rows, waterfall_image.cols, waterfall_image.type());
    model_waterfall_image(cv::Rect(0, 0, waterfall_image.cols, waterfall_image.rows-1)).copyTo(shifted(cv::Rect(0, 1, shifted.cols, shifted.rows-1)));
    shifted.copyTo(model_waterfall_image);

    Eigen::Vector3d pos = pings[i].pos_ - offset;

    Eigen::VectorXd intensities_left = compute_model_intensities(hits_left, normals_left, pos);
    Eigen::VectorXd intensities_right = compute_model_intensities(hits_right, normals_right, pos);

    double ping_step = pings[i].port.time_duration / double(nbr_windows);
    for (int j = 0; j < times_left.rows(); ++j) {
        int index = int(times_left(j)/ping_step);
        if (index < nbr_windows) {
            model_waterfall_image.at<uint8_t>(0, nbr_windows-index-1) = uint8_t(255.*intensities_left(j));
        }
    }

    for (int j = 0; j < times_right.rows(); ++j) {
        int index = int(times_right(j)/ping_step);
        if (index < nbr_windows) {
            model_waterfall_image.at<uint8_t>(0, nbr_windows+index) = uint8_t(255.*intensities_right(j));
        }
    }
}

// NOTE: sss_ping_duration should be the same as for the pings
Eigen::MatrixXd SSSGenSim::draw_sim_waterfall(const Eigen::MatrixXd& incidence_image)
{
    Eigen::MatrixXd sim_image = Eigen::MatrixXd::Zero(incidence_image.rows(), incidence_image.cols());

    int overlap = (full_window_height - resample_window_height)/2;

    //for (int k = overlap; k + resample_window_height + overlap < incidence_image.rows(); k += resample_window_height) {
    for (int k = incidence_image.rows() - full_window_height; k > 0; k -= resample_window_height) {
        Eigen::MatrixXd model_image = incidence_image.middleRows(k-overlap, full_window_height);
        Eigen::MatrixXd generated = gen_callback(model_image);
        for (int row = 0; row < resample_window_height; ++row) {
            int image_row = k + row;
            int offset_row = row + overlap;
            for (int col = 0; col < generated.cols(); ++col) {
                sim_image(image_row, col) = generated(offset_row, col);
            }
        }
        // this interpolates an area between the previous and current windows
        for (int row = 0; row < overlap; ++row) {
            int image_row = k + resample_window_height + row;
            int offset_row = row + resample_window_height + overlap;
            for (int col = 0; col < generated.cols(); ++col) {
                sim_image(image_row, col) = generated(offset_row, col)*(1.-double(row)/double(overlap)) + sim_image(image_row, col)*double(row)/double(overlap);
            }
        }
    }

    return sim_image;
}

// here we already have the SVP so should be fine
Eigen::MatrixXd SSSGenSim::draw_model_waterfall(const Eigen::MatrixXd& incidence_image, double sss_ping_duration)
{
    int ping_side_windows = incidence_image.cols()/2;
    double sound_vel = sound_speeds[0].vels.head(sound_speeds[0].vels.rows()-1).mean();

    Eigen::VectorXd dists(ping_side_windows);
    for (int k = 0; k < ping_side_windows; ++k) {
        dists(k) = 0.5*double(k+1)/double(ping_side_windows)*sss_ping_duration*sound_vel;
    }

    Eigen::MatrixXd model_image = Eigen::MatrixXd::Zero(incidence_image.rows(), incidence_image.cols());

    for (int j = 0; j < incidence_image.rows(); ++j) {

        Eigen::VectorXd thetas_left(ping_side_windows);
        Eigen::VectorXd thetas_right(ping_side_windows);

        for (int k = 0; k < ping_side_windows; ++k) {
            thetas_left(k) = acos(sqrt(incidence_image(j, ping_side_windows-1-k)));
            thetas_right(k) = acos(sqrt(incidence_image(j, ping_side_windows+k)));
        }

        Eigen::VectorXd intensities_left = compute_model_intensities(dists, thetas_left);
        Eigen::VectorXd intensities_right = compute_model_intensities(dists, thetas_right);

        model_image.block(j, 0, 1, ping_side_windows) = intensities_left.reverse().transpose();
        model_image.block(j, ping_side_windows, 1, ping_side_windows) = intensities_right.transpose();
    }

    return model_image;
}

bool SSSGenSim::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    //while (!is_mesh_underneath_vehicle(pings[i].pos_ - offset, V1, F1) && i < pings.size()) {
    while (!fast_is_mesh_underneath_vehicle(pings[i].pos_ - offset) && i < pings.size()) {
        i += 10;
    }

    if (i >= pings.size()) {
        return false;
    }

    bool prev_sss = false;

    if (sss_from_waterfall) {
        if (waterfall_row == resample_window_height) {
            //Eigen::MatrixXd generated = gen_callback(waterfall_depth);
            
            Eigen::MatrixXd generated = gen_callback(waterfall_model);

            double left_bottom_row_mean = generated.row(resample_window_height-1).head(generated.cols()/2).mean();
            double right_bottom_row_mean = generated.row(resample_window_height-1).tail(generated.cols()/2).mean();
            for (int row = 0; row < resample_window_height; ++row) {
                int offset_row = row + (full_window_height - resample_window_height) / 2;
                for (int col = 0; col < generated.cols()/2; ++col) {
                    //double correction = 1./double(resample_window_height)*(double(resample_window_height - row) + double(row)*left_row_mean/left_bottom_row_mean);
                    double correction = 1.;
                    //waterfall_image.at<uint8_t>(row, col) = uint8_t(255.*correction*generated(row, col));
                    waterfall_image.at<uint8_t>(offset_row, col) = uint8_t(255.*correction*generated(offset_row, col));
                }
                for (int col = generated.cols()/2; col < generated.cols(); ++col) {
                    //double correction = 1./double(resample_window_height)*(double(resample_window_height - row) + double(row)*right_row_mean/right_bottom_row_mean);
                    double correction = 1.;
                    //waterfall_image.at<uint8_t>(row, col) = uint8_t(255.*correction*generated(row, col));
                    waterfall_image.at<uint8_t>(offset_row, col) = uint8_t(255.*correction*generated(offset_row, col));
                }
            }
            // this interpolates an area between the previous and current windows
            for (int row = 0; row < (full_window_height - resample_window_height)/2; ++row) {
                int offset_row = row + resample_window_height + (full_window_height-resample_window_height)/2;
                double full = (full_window_height-resample_window_height)/2;
                for (int col = 0; col < generated.cols(); ++col) {
                    waterfall_image.at<uint8_t>(offset_row, col) = uint8_t(255.*generated(offset_row, col)*(1.-double(row)/full) + double(waterfall_image.at<uint8_t>(offset_row, col))*double(row)/full);
                }
            }
            left_row_mean = generated.row(0).head(generated.cols()/2).mean();
            right_row_mean = generated.row(0).tail(generated.cols()/2).mean();
            if (full_window_height > resample_window_height) {
                Eigen::MatrixXd temp = waterfall_depth.topRows(full_window_height-resample_window_height);
                waterfall_depth.bottomRows(full_window_height-resample_window_height) = temp;
                waterfall_depth.topRows(resample_window_height).setZero();
                temp = waterfall_model.topRows(full_window_height-resample_window_height);
                waterfall_model.bottomRows(full_window_height-resample_window_height) = temp;
                waterfall_model.topRows(resample_window_height).setZero();
                if (prev_sss) {
                    waterfall_model.bottomRows(full_window_height-resample_window_height) = generated.topRows(full_window_height-resample_window_height);
                }
            }
            waterfall_row = 0;
        }
    }
    else if (sss_from_bathy) {
        if ((window_point - (pings[i].pos_ - offset)).norm() > 4.) {
            window_point = pings[i].pos_ - offset;
            window_heading = pings[i].heading_ + sensor_yaw;
            generate_sss_window();
        }
    }

    // compute 3d hits and normals
    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    Eigen::MatrixXd normals_left;
    Eigen::MatrixXd normals_right;
    tie(hits_left, hits_right, normals_left, normals_right) = project(pings[i]);

    // compute travel times
    Eigen::Vector3d origin_port;
    Eigen::Vector3d origin_stbd;
    tie(origin_port, origin_stbd) = get_port_stbd_sensor_origins(pings[i]);
    Eigen::VectorXd times_left = compute_times(origin_port, hits_left);
    Eigen::VectorXd times_right = compute_times(origin_stbd, hits_right);

    // compute the intensity values for vis
    Eigen::VectorXd gt_intensities_left = compute_intensities(times_left, pings[i].port);
    Eigen::VectorXd gt_intensities_right = compute_intensities(times_right, pings[i].stbd);
    add_texture_intensities(hits_left, gt_intensities_left);
    add_texture_intensities(hits_right, gt_intensities_right);

    // shift the waterfall image
    cv::Mat shifted = cv::Mat::zeros(waterfall_image.rows, waterfall_image.cols, waterfall_image.type());
    waterfall_image(cv::Rect(0, 0, waterfall_image.cols, waterfall_image.rows-1)).copyTo(shifted(cv::Rect(0, 1, shifted.cols, shifted.rows-1)));
    shifted.copyTo(waterfall_image);

    if (sss_from_waterfall) {
    
        Eigen::Vector3d pos = pings[i].pos_ - offset;

        // maybe do this outside the statement and reuse them for the cv image generation
        Eigen::VectorXd model_windows_left = convert_to_time_bins(times_left, compute_lambert_intensities(hits_left, normals_left, pos), pings[i].port, nbr_windows);
        Eigen::VectorXd model_windows_right = convert_to_time_bins(times_right, compute_lambert_intensities(hits_right, normals_right, pos), pings[i].stbd, nbr_windows);

        Eigen::VectorXd model_windows(model_windows_left.rows() + model_windows_right.rows());
        model_windows.tail(model_windows_right.rows()) = model_windows_right;
        model_windows.head(model_windows_left.rows()) = model_windows_left.reverse();

        waterfall_model.row(resample_window_height-waterfall_row-1) = model_windows.transpose();

        Eigen::VectorXd depth_windows_left = convert_to_time_bins(times_left, Eigen::VectorXd(hits_left.col(2)), pings[i].port, nbr_windows);
        Eigen::VectorXd depth_windows_right = convert_to_time_bins(times_right, Eigen::VectorXd(hits_right.col(2)), pings[i].stbd, nbr_windows);

        Eigen::VectorXd depth_windows(depth_windows_left.rows() + depth_windows_right.rows());
        depth_windows.tail(depth_windows_right.rows()) = depth_windows_right;
        depth_windows.head(depth_windows_left.rows()) = depth_windows_left.reverse();

        waterfall_depth.row(resample_window_height-waterfall_row-1) = depth_windows.transpose();
        ++waterfall_row;

    }
    else if (sss_from_bathy) {

        Eigen::VectorXd intensities_left = get_texture_intensities(hits_left);
        Eigen::VectorXd intensities_right = get_texture_intensities(hits_right);

        Eigen::VectorXd time_windows_left = convert_to_time_bins(times_left, intensities_left, pings[i].port, nbr_windows);
        Eigen::VectorXd time_windows_right = convert_to_time_bins(times_right, intensities_right, pings[i].stbd, nbr_windows);

        Eigen::VectorXd time_windows(time_windows_left.rows() + time_windows_right.rows());
        time_windows.tail(time_windows_right.rows()) = time_windows_right;
        time_windows.head(time_windows_left.rows()) = time_windows_left.reverse();

        for (int i = 0; i < std::min(time_windows.rows(), int64_t(waterfall_image.cols)); ++i) {
            waterfall_image.at<uint8_t>(0, i) = uint8_t(255.*time_windows(i));
        }

    }

    // construct the ground truth waterfall image for timestep
    construct_gt_waterfall();

    // construct the model waterfall image for timestep
    construct_model_waterfall(hits_left, hits_right, normals_left, normals_right, times_left, times_right);

    if (sss_from_bathy || sss_from_waterfall) {
        cv::imshow("GAN waterfall image", waterfall_image);
    }

    cv::imshow("Ground truth waterfall image", gt_waterfall_image);

    cv::imshow("Model waterfall image", model_waterfall_image);

    if (true) {
        cv::Mat compare_waterfall_image;
        vector<cv::Mat> channels;
        cv::Mat b = cv::Mat::zeros(1000, 2*nbr_windows, CV_8UC1);

        channels.push_back(b);
        channels.push_back(gt_waterfall_image);
        channels.push_back(model_waterfall_image);

        cv::merge(channels, compare_waterfall_image);
        cv::imshow("Compare waterfall image", compare_waterfall_image);
    }

    cv::waitKey(10);

    if (i % 10 == 0) {
        //visualize_vehicle();
        //visualize_rays(hits_left, hits_right);
        visualize_rays(origin_port, hits_left, true);
        visualize_rays(origin_stbd, hits_right);
        visualize_intensities();
    }

    ++i;

    return false;
}
