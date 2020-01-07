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

#include <bathy_maps/patch_draper.h>

#include <igl/readSTL.h>
#include <igl/unproject_onto_mesh.h>
//#include <bathy_maps/drape_mesh.h>

using namespace std;
using namespace csv_data;

PatchDraper::PatchDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                         const std_data::sss_ping::PingsT& pings,
                         const BoundsT& bounds,
                         const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : ViewDraper(V1, F1, pings, bounds, sound_speeds), save_callback(&default_callback)
{
    is_active = Eigen::VectorXi(pings.size()); is_active.setOnes();

    viewer.callback_mouse_down = std::bind(&PatchDraper::callback_mouse_down, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    viewer.callback_pre_draw = std::bind(&PatchDraper::callback_pre_draw, this, std::placeholders::_1);
}

/*
void PatchDraper::handle_patches()
{
    int skipped = 0;
    for (; i < pings.size() && is_active[i] == 0; ++i, ++skipped) {}
    if (patch_assembler.is_active() && skipped > 0) {
        patch_assembler.split();
    }

    if (i >= pings.size()) {
        if (patch_assembler.is_active() && !patch_assembler.empty()) {
            patch_views.push_back(patch_assembler.finish());
            save_callback(patch_views.back());
        }
        return;
    }

    Eigen::MatrixXd hits_left_intensities, hits_right_intensities;
    Eigen::VectorXi hits_left_pings_indices, hits_right_pings_indices;
    Eigen::Vector3d pos;
    tie(hits_left_intensities, hits_right_intensities, hits_left_pings_indices, hits_right_pings_indices, pos) = project_sss();

    patch_assembler.add_hits(hits_left_intensities, pos);
    patch_assembler.add_hits(hits_right_intensities, pos);
}
*/

bool PatchDraper::point_in_view(const std_data::sss_ping& ping, const Eigen::Vector3d& point, double sensor_yaw)
{
    //Eigen::Matrix3d Ry = Eigen::AngleAxisd(ping.pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Rcomp; //*Ry;

    // first, let's transform the point to a coordinate system defined by the sonar
    Eigen::Vector3d p = R.transpose()*(point - ping.pos_);

    // now, let's get the yaw and pitch components
    double yaw = atan2(p(1), p(0));

    double xy_dist = fabs(p(1)); //sqrt(p(1)*p(1)+p(0)*p(0));
    double pitch = atan(p(2)/xy_dist);

    double min_pitch = -1.4*ping.port.tilt_angle - 0.5*ping.port.beam_width;
    double max_pitch = -1.4*ping.port.tilt_angle + 0.5*ping.port.beam_width - M_PI/20.;

    // check if point is in view of either of the side scans
    //bool yaw_in_view = fabs(yaw) < M_PI/2. + M_PI/16. && fabs(yaw) > M_PI/2. - M_PI/16.;
    bool yaw_in_view = fabs(p(0)) < 5.;

    bool pitch_in_view = pitch < max_pitch && pitch > min_pitch;

    cout << "Pitch: " << pitch << ", min pitch: " << min_pitch << ", max pitch: " << max_pitch << endl;

    cout << "Pitch in view?: " << pitch_in_view << " and yaw in view?: " << yaw_in_view << endl;

    cout << "XTF pos height: " << ping.pos_(2) << endl;
    cout << "Clicked oint height: " << point(2) << endl;
    cout << "Pos height: " << p(2) << endl;

    return pitch_in_view && yaw_in_view;
}

void PatchDraper::handle_patches()
{

    int skipped = 0;
    for (; i < pings.size() && is_active[i] == 0; ++i, ++skipped) {}
    if (patch_assembler.is_active() && skipped > 0) {
        patch_assembler.split();

        // NOTE: this could probably be part of preceding if statement
        if (false) {//!patch_assembler.empty()) {
            Eigen::MatrixXd patch_map = patch_assembler.get_last_patch_view();
            //patch_map.array() *= (patch_map.array() <= 0).cast<double>();
            double world_size = patch_assembler.get_world_size();
            Eigen::Vector3d patch_origin = patch_assembler.get_origin();
            BoundsT bounds; bounds << patch_origin(0) - .5*world_size, patch_origin(1) - .5*world_size, patch_origin(0) + .5*world_size, patch_origin(1) + .5*world_size;
            set_texture(patch_map, bounds);
        }
    }

    if (i >= pings.size()) {
        if (patch_assembler.is_active() && !patch_assembler.empty()) {
            patch_views.push_back(patch_assembler.finish());
            save_callback(patch_views.back());
        }
        return;
    }

    Eigen::Vector3d pos = pings[i].pos_ - offset;

    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    Eigen::MatrixXd normals_left;
    Eigen::MatrixXd normals_right;
    tie(hits_left, hits_right, normals_left, normals_right) = project(pings[i]);

    // these should take care of computing bending if set
    Eigen::Vector3d origin_port;
    Eigen::Vector3d origin_stbd;
    tie(origin_port, origin_stbd) = get_port_stbd_sensor_origins(pings[i]);
    Eigen::VectorXd times_left = compute_times(origin_port, hits_left);
    Eigen::VectorXd times_right = compute_times(origin_stbd, hits_right);

    // compute the ground truth intensities
    Eigen::VectorXd intensities_left = compute_intensities(times_left, pings[i].port);
    Eigen::VectorXd intensities_right = compute_intensities(times_right, pings[i].stbd);

    patch_assembler.add_hits(hits_left, intensities_left, pos);
    patch_assembler.add_hits(hits_right, intensities_right, pos);

    if (i % 10 == 0) {
        visualize_vehicle();
        //visualize_rays(hits_left, hits_right);
        visualize_rays(origin_port, hits_left, true);
        visualize_rays(origin_stbd, hits_right);
    }

    i += 1;
}

bool PatchDraper::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    handle_patches();
    return false;
}

bool PatchDraper::callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int)
{
    cout << "Got mouse callback!" << endl;
    int fid;
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
            viewer.core().proj, viewer.core().viewport, V1, F1, fid, bc)) {
        // paint hit red
        cout << "Got point in mesh!" << endl;
        int vind = F1(fid, 0);
        C.row(vind) << 1, 0, 0;
        viewer.data().set_colors(C);
        Eigen::Vector3d point = offset + V1.row(vind).transpose();
        int nbr_in_view = 0;
        for (int j = 0; j < pings.size(); ++j) {
            is_active(j) = int(point_in_view(pings[j], point, sensor_yaw));
            nbr_in_view += is_active(j);
        }
        i = 0;

        if (patch_assembler.is_active() && !patch_assembler.empty()) {
            patch_views.push_back(patch_assembler.finish());
        }
        patch_assembler = sss_patch_assembler();
        patch_assembler.activate(point - offset);

        cout << "Number in view: " << nbr_in_view << " out of: " << pings.size() << endl;

        return true;
    }
    cout << "Not in mesh!" << endl;
    return false;
}

bool PatchDraper::callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods)
{
    switch (key) {
    case 'n':
        handle_patches();
        i += 1;
        return true;
    case 'm':
        handle_patches();
        i += 100;
        return true;
    default:
        return false;
    }
}

sss_patch_views::ViewsT PatchDraper::get_patch_views()
{
    return patch_views;
}

sss_patch_views::ViewsT drape_patches(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                      const PatchDraper::BoundsT& bounds, const std_data::sss_ping::PingsT& pings,
                                      const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                      const std::function<void(sss_patch_views)>& save_callback)
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Cb;
    tie(Vb, Fb, Cb) = get_vehicle_mesh();

    PatchDraper viewer(V, F, pings, bounds, sound_speeds);
    viewer.set_sidescan_yaw(sensor_yaw);
    viewer.set_patch_callback(save_callback);
    viewer.set_vehicle_mesh(Vb, Fb, Cb);
    viewer.show();

    sss_patch_views::ViewsT views = viewer.get_patch_views();

    cout << "Got " << views.size() << " patch views..." << endl;

    return views;
}
