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

MapDraper::MapDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                     const xtf_sss_ping::PingsT& pings,
                     const BoundsT& bounds,
                     const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : BaseDraper(V1, F1, pings, bounds, sound_speeds),
      bounds(bounds), resolution(30./8.), save_callback(&default_callback),
      map_image_builder(bounds, 30./8., pings[0].port.pings.size())
{
    viewer.callback_pre_draw = std::bind(&MapDraper::callback_pre_draw, this, std::placeholders::_1);
    int rows, cols;
    tie(rows, cols) = map_image_builder.get_map_image_shape();
    draping_vis_texture = Eigen::MatrixXd::Zero(rows, cols);
}

bool MapDraper::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    // check if ping is first_in_file_, in that case, split off new image
    if ((i == pings.size() - 1 || (i+1 < pings.size() && pings[i+1].first_in_file_)) && !map_image_builder.empty()) {
        sss_map_image map_image = map_image_builder.finish();
        draping_vis_texture.array() *= (map_image.sss_map_image.array() == 0).cast<double>();
        draping_vis_texture.array() += map_image.sss_map_image.array();
        Eigen::MatrixXd texture = draping_vis_texture;
        texture.array() += (texture.array() == 0).cast<double>();
        set_texture(texture, bounds);
        save_callback(map_image);
        map_images.push_back(map_image);
        map_image_builder = sss_map_image_builder(bounds, resolution, pings[i].port.pings.size());
    }

    if (i >= pings.size()) {
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

    // add the 3d hits and waterfall images to the builder object
    map_image_builder.add_hits(hits_left, intensities_left, sss_depths_left, sss_model_left, pings[i].port, pos, true);
    map_image_builder.add_hits(hits_right, intensities_right, sss_depths_right, sss_model_right, pings[i].stbd, pos, false);

    if (i % 10 == 0) {
        visualize_rays(hits_left, hits_right);
        visualize_vehicle();
    }

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

void MapDraper::set_resolution(double new_resolution)
{ 
    resolution = new_resolution;
    map_image_builder = sss_map_image_builder(bounds, resolution, pings[i].port.pings.size());
    int rows, cols;
    tie(rows, cols) = map_image_builder.get_map_image_shape();
    draping_vis_texture = Eigen::MatrixXd::Zero(rows, cols);
}

sss_map_image::ImagesT MapDraper::get_images()
{
    return map_images;
}

sss_map_image::ImagesT drape_maps(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                  const MapDraper::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                                  const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                  double resolution, const std::function<void(sss_map_image)>& save_callback)
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Cb;
    tie(Vb, Fb, Cb) = get_vehicle_mesh();

    Eigen::MatrixXd C_jet = color_jet_from_mesh(V);

    MapDraper viewer(V, F, pings, bounds, sound_speeds);
    viewer.set_sidescan_yaw(sensor_yaw);
    viewer.set_resolution(resolution);
    viewer.set_image_callback(save_callback);
    viewer.set_vehicle_mesh(Vb, Fb, Cb);
    viewer.show();

    return viewer.get_images();
}
