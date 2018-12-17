#ifndef DRAPING_VIEWER_H
#define DRAPING_VIEWER_H

#include <bathy_maps/draping_generator.h>
#include <bathy_maps/patch_views.h>

struct survey_viewer : public draping_generator {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    sss_patch_assembler patch_assembler;
    sss_patch_views::ViewsT patch_views;
    Eigen::VectorXi is_active;
    const std::function<void(sss_patch_views)>& save_callback;

public:

    static void default_callback(const sss_patch_views&) {}

    survey_viewer(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
        const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
        const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset,
        const csv_asvp_sound_speed::EntriesT& sound_speeds = csv_asvp_sound_speed::EntriesT(),
        double sensor_yaw = 0., const std::function<void(sss_patch_views)>& save_callback = &default_callback);

    void handle_patches();
    bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int);
    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods);
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
    sss_patch_views::ViewsT get_patch_views();
};

sss_patch_views::ViewsT overlay_sss(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                    const survey_viewer::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                                    const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                    const std::function<void(sss_patch_views)>& save_callback = &survey_viewer::default_callback);

#endif // DRAPING_VIEWER_H
