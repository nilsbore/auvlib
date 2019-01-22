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
      height_map(height_map)
{
    viewer.callback_pre_draw = std::bind(&SSSGenSim::callback_pre_draw, this, std::placeholders::_1);
    window_point = Eigen::Vector3d::Zero();
    height_map_cv = cv::Mat(height_map.rows(), height_map.cols(), CV_32FC1);
    for (int i = 0; i < height_map.rows(); ++i) {
        for (int j = 0; j < height_map.cols(); ++j) {
            height_map_cv.at<float>(i, j) = height_map(i, j);
        }
    }
}

Eigen::MatrixXd scale_height_map(const Eigen::MatrixXd& height_map)
{
    Eigen::ArrayXXd height_map_array = height_map.array();
    double minv = height_map.minCoeff();
    height_map_array -= minv*(height_map_array < 0).cast<double>();
    double maxv = height_map_array.maxCoeff();
    height_map_array /= maxv;
    return height_map.matrix();
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

    double heading = 180./M_PI*pings[i].heading_;

    //resolution = (bounds[1, 0] - bounds[0, 0])/1333.
    double resolution = double(height_map_cv.cols)/(bounds(1, 0) - bounds(0, 0));
    Eigen::Vector3d pos = resolution*window_point;

    cv::Size2f cv_image_size = cv::Size2f(20., 10*20.);
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

    //cv::Mat roi = height_map_cv(brect);
    cv::Mat roi = vis_image(brect);

    cv::Point center = cv::Point(roi.cols/2, roi.rows/2);

    cv::Mat rot = cv::getRotationMatrix2D(center, heading, 1.0);

    cv::Mat rotated;
    cv::warpAffine(roi, rotated, rot, roi.size(), cv::INTER_CUBIC);

    cv::Point rotcenter = cv::Point(rotated.cols/2, rotated.rows/2);
    cv::Mat cropped;
    cv::getRectSubPix(rotated, cv_image_size, rotcenter, cropped);

    cv::imshow("Cropped", cropped);
    cv::waitKey(10);
}

bool SSSGenSim::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    if (i >= pings.size()) {
        return false;
    }

    if ((window_point - (pings[i].pos_ - offset)).norm() > 7.) {
        generate_sss_window();
        window_point = pings[i].pos_ - offset;
    }

    Eigen::MatrixXd hits_left_intensities, hits_right_intensities;
    Eigen::VectorXi hits_left_pings_indices, hits_right_pings_indices;
    Eigen::Vector3d pos;
    tie(hits_left_intensities, hits_right_intensities, hits_left_pings_indices, hits_right_pings_indices, pos) = project_sss();

    ++i;

    return false;
}

/*
void SSSGenSim::set_resolution(double new_resolution)
{ 
    resolution = new_resolution;
    map_image_builder = sss_map_image_builder(bounds, resolution, pings[i].port.pings.size());
}
*/

/*
sss_map_image::ImagesT drape_maps(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                  const SSSGenSim::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                                  const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                  double resolution, const std::function<void(sss_map_image)>& save_callback)
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Cb;
    tie(Vb, Fb, Cb) = get_vehicle_mesh();

    Eigen::MatrixXd C_jet = color_jet_from_mesh(V);

    SSSGenSim viewer(V, F, pings, bounds, sound_speeds);
    viewer.set_sidescan_yaw(sensor_yaw);
    viewer.set_resolution(resolution);
    viewer.set_image_callback(save_callback);
    viewer.set_vehicle_mesh(Vb, Fb, Cb);
    viewer.show();

    return viewer.get_images();
}
*/
