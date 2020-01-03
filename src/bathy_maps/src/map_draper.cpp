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
#include <bathy_maps/impl/map_draper.hpp>
#include <bathy_maps/sss_meas_data.h>

template class MapDraper<sss_map_image_builder>;

template class MapDraper<sss_meas_data_builder>;

sss_map_image::ImagesT drape_maps(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                  const BaseDraper::BoundsT& bounds, const std_data::sss_ping::PingsT& pings,
                                  const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                  double resolution, const std::function<void(sss_map_image)>& save_callback)
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Cb;
    tie(Vb, Fb, Cb) = get_vehicle_mesh();

    Eigen::MatrixXd C_jet = color_jet_from_mesh(V);

    MapDraper<sss_map_image_builder> viewer(V, F, pings, bounds, sound_speeds);
    viewer.set_sidescan_yaw(sensor_yaw);
    viewer.set_resolution(resolution);
    viewer.set_image_callback(save_callback);
    viewer.set_vehicle_mesh(Vb, Fb, Cb);
    viewer.show();

    return viewer.get_images();
}
