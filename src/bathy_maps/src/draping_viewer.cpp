#include <bathy_maps/draping_viewer.h>

#include <igl/readSTL.h>
#include <igl/unproject_onto_mesh.h>
#include <bathy_maps/drape_mesh.h>

using namespace std;

draping_patches::draping_patches(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                                 const xtf_sss_ping::PingsT& pings,
                                 const BoundsT& bounds,
                                 const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : draping_generator(V1, F1, pings, bounds, sound_speeds), save_callback(&default_callback)
{
    is_active = Eigen::VectorXi(pings.size()); is_active.setOnes();

    viewer.callback_mouse_down = std::bind(&draping_patches::callback_mouse_down, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    viewer.callback_pre_draw = std::bind(&draping_patches::callback_pre_draw, this, std::placeholders::_1);
}

void draping_patches::handle_patches()
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

bool draping_patches::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    handle_patches();
    i += 1;
    return false;
}

bool draping_patches::callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int)
{
    cout << "Got mouse callback!" << endl;
    int fid;
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core.viewport(3) - viewer.current_mouse_y;
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view * viewer.core.model,
            viewer.core.proj, viewer.core.viewport, V1, F1, fid, bc)) {
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

bool draping_patches::callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods)
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

sss_patch_views::ViewsT draping_patches::get_patch_views()
{
    return patch_views;
}

sss_patch_views::ViewsT overlay_sss(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                    const draping_patches::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                                    const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                    const std::function<void(sss_patch_views)>& save_callback)
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Cb;
    tie(Vb, Fb, Cb) = get_vehicle_mesh();

    draping_patches viewer(V, F, pings, bounds, sound_speeds);
    viewer.set_sidescan_yaw(sensor_yaw);
    viewer.set_patch_callback(save_callback);
    viewer.set_vehicle_mesh(Vb, Fb, Cb);
    viewer.show();

    sss_patch_views::ViewsT views = viewer.get_patch_views();

    cout << "Got " << views.size() << " patch views..." << endl;

    return views;
}
