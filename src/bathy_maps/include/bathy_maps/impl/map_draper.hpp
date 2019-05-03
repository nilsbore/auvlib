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

#include <bathy_maps/map_draper.h>

#include <igl/readSTL.h>
#include <igl/unproject_onto_mesh.h>
#include <bathy_maps/drape_mesh.h>

using namespace std;
using namespace xtf_data;
using namespace csv_data;

template <typename MapSaver>
MapDraper<MapSaver>::MapDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                               const xtf_sss_ping::PingsT& pings,
                               const BoundsT& bounds,
                               const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : BaseDraper(V1, F1, pings, bounds, sound_speeds),
      resolution(30./8.), save_callback(&default_callback),
      map_image_builder(bounds, 30./8., 256)
      //map_image_builder(bounds, 30./8., pings[0].port.pings.size())
{
    viewer.callback_pre_draw = std::bind(&MapDraper::callback_pre_draw, this, std::placeholders::_1);
    //int rows, cols;
    //tie(rows, cols) = map_image_builder.get_map_image_shape();
    //draping_vis_texture = Eigen::MatrixXd::Zero(rows, cols);
    store_map_images = true;
}

template <typename MapSaver>
bool MapDraper<MapSaver>::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    cout << "Start callback_pre_draw" << endl;

    //while ((i+1 < pings.size() && !pings[i+1].first_in_file_) && !is_mesh_underneath_vehicle(pings[i].pos_ - offset, V1, F1)) {
    while ((i+1 < pings.size() && !pings[i+1].first_in_file_) && !fast_is_mesh_underneath_vehicle(pings[i].pos_ - offset)) {
        i += 1;
    }

    cout << "Save?" << endl;

    // check if ping is first_in_file_, in that case, split off new image
    if ((i == pings.size() - 1 || (i+1 < pings.size() && pings[i+1].first_in_file_)) && !map_image_builder.empty()) {
        cout << "Yes" << endl;
        MapType map_image = map_image_builder.finish();
        /*
        draping_vis_texture.array() *= (map_image.sss_map_image.array() == 0).cast<double>();
        // TODO: make the intensity multiplier a parameter
        //draping_vis_texture.array() += map_image.sss_map_image.array();
        draping_vis_texture.array() += 2.*map_image.sss_map_image.array();
        Eigen::MatrixXd texture = draping_vis_texture;
        texture.array() += (texture.array() == 0).cast<double>();
        set_texture(texture, bounds);
        */
        visualize_intensities();
        save_callback(map_image);
        if (store_map_images) {
            map_images.push_back(map_image);
        }
        //map_image_builder = sss_map_image_builder(bounds, resolution, pings[i].port.pings.size());
        map_image_builder = MapSaver(bounds, resolution, 256);
    }

    if (i >= pings.size()) {
        glfwSetWindowShouldClose(viewer.window, 1);
        return false;
    }

    Eigen::Vector3d pos = pings[i].pos_ - offset;

    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    Eigen::MatrixXd normals_left;
    Eigen::MatrixXd normals_right;
    tie(hits_left, hits_right, normals_left, normals_right) = project();

    // these should take care of computing bending if set
    Eigen::VectorXd times_left = compute_times(hits_left);
    Eigen::VectorXd times_right = compute_times(hits_right);

    // compute the elevation waterfall row
    Eigen::VectorXd sss_depths_left = convert_to_time_bins(times_left, hits_left.col(2), pings[i].port, map_image_builder.get_waterfall_bins());
    Eigen::VectorXd sss_depths_right = convert_to_time_bins(times_right, hits_right.col(2), pings[i].stbd, map_image_builder.get_waterfall_bins());

    // compute the intensities of the model
    Eigen::VectorXd model_intensities_left = compute_model_intensities(hits_left, normals_left, pos);
    Eigen::VectorXd model_intensities_right = compute_model_intensities(hits_right, normals_right, pos);
    Eigen::VectorXd sss_model_left = convert_to_time_bins(times_left, model_intensities_left, pings[i].port, map_image_builder.get_waterfall_bins());
    Eigen::VectorXd sss_model_right = convert_to_time_bins(times_right, model_intensities_right, pings[i].stbd, map_image_builder.get_waterfall_bins());

    // compute the ground truth intensities
    Eigen::VectorXd intensities_left = compute_intensities(times_left, pings[i].port);
    Eigen::VectorXd intensities_right = compute_intensities(times_right, pings[i].stbd);

    // add intensities for visualization
    add_texture_intensities(hits_left, intensities_left);
    add_texture_intensities(hits_right, intensities_right);

    // compute waterfall image inds of hits
    Eigen::VectorXi hits_inds_left = compute_bin_indices(times_left, pings[i].port, map_image_builder.get_waterfall_bins());
    Eigen::VectorXi hits_inds_right = compute_bin_indices(times_right, pings[i].stbd, map_image_builder.get_waterfall_bins());

    cout << "Adding hits" << endl;

    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Vector3d rpy(pings[i].roll_, pings[i].pitch_, pings[i].heading_);
    // add the 3d hits and waterfall images to the builder object
    map_image_builder.add_hits(hits_left, hits_inds_left, intensities_left, sss_depths_left, sss_model_left, pings[i].port, pos, rpy, true);
    map_image_builder.add_hits(hits_right, hits_inds_right, intensities_right, sss_depths_right, sss_model_right, pings[i].stbd, pos, rpy, false);

    cout << "Done adding hits, visualizing" << endl;

    if (i % 10 == 0) {
        visualize_vehicle();
        visualize_rays(hits_left, hits_right);
    }

    cout << "Done visualizing" << endl;

    ++i;

    return false;
}

/*
bool MapDraper::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    // check if ping is first_in_file_, in that case, split off new image
    if ((i == pings.size() - 1 || (i+1 < pings.size() && pings[i+1].first_in_file_)) && !map_image_builder.empty()) {
        sss_map_image map_image = map_image_builder.finish();
        save_callback(map_image);
        map_images.push_back(map_image);
        map_image_builder = sss_map_image_builder(bounds, resolution, pings[i].port.pings.size());
    }

    if (i >= pings.size()) {
        return false;
    }

    Eigen::MatrixXd hits_left_intensities, hits_right_intensities;
    Eigen::VectorXi hits_left_pings_indices, hits_right_pings_indices;
    Eigen::Vector3d pos;
    tie(hits_left_intensities, hits_right_intensities, hits_left_pings_indices, hits_right_pings_indices, pos) = project_sss();

    map_image_builder.add_hits(hits_left_intensities, hits_left_pings_indices, pings[i].port, pos, true);
    map_image_builder.add_hits(hits_right_intensities, hits_right_pings_indices, pings[i].stbd, pos, false);

    ++i;

    return false;
}
*/

template <typename MapSaver>
void MapDraper<MapSaver>::set_resolution(double new_resolution)
{ 
    resolution = new_resolution;
    //map_image_builder = sss_map_image_builder(bounds, resolution, pings[i].port.pings.size());
    map_image_builder = MapSaver(bounds, resolution, 256);
    //int rows, cols;
    //tie(rows, cols) = map_image_builder.get_map_image_shape();
    //draping_vis_texture = Eigen::MatrixXd::Zero(rows, cols);
}

template <typename MapSaver>
typename MapSaver::MapType::ImagesT MapDraper<MapSaver>::get_images()
{
    return map_images;
}
