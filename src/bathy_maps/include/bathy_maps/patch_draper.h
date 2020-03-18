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

#ifndef PATCH_DRAPER_H
#define PATCH_DRAPER_H

#include <bathy_maps/view_draper.h>
#include <bathy_maps/patch_views.h>

struct PatchDraper : public ViewDraper {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    sss_patch_assembler patch_assembler;
    sss_patch_views::ViewsT patch_views;
    Eigen::VectorXi is_active;
    std::function<void(sss_patch_views)> save_callback;

public:

    static void default_callback(const sss_patch_views&) {}
    
    void set_patch_callback(const std::function<void(sss_patch_views)>& callback) { save_callback = callback; }

    PatchDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                const std_data::sss_ping::PingsT& pings,
                const BoundsT& bounds,
                const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds);

    void handle_patches();
    bool point_in_view(const std_data::sss_ping& ping, const Eigen::Vector3d& point, double sensor_yaw);
    bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int);
    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods);
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
    sss_patch_views::ViewsT get_patch_views();
};

sss_patch_views::ViewsT drape_patches(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                      const PatchDraper::BoundsT& bounds, const std_data::sss_ping::PingsT& pings,
                                      const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                      const std::function<void(sss_patch_views)>& save_callback = &PatchDraper::default_callback);

#endif // PATCH_DRAPER_H
