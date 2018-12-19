#ifndef PATCH_DRAPER_H
#define PATCH_DRAPER_H

#include <bathy_maps/base_draper.h>
#include <bathy_maps/patch_views.h>

struct PatchDraper : public BaseDraper {
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
                const xtf_data::xtf_sss_ping::PingsT& pings,
                const BoundsT& bounds,
                const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds);

    void handle_patches();
    bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int);
    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods);
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
    sss_patch_views::ViewsT get_patch_views();
};

sss_patch_views::ViewsT drape_patches(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                      const PatchDraper::BoundsT& bounds, const xtf_data::xtf_sss_ping::PingsT& pings,
                                      const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                                      const std::function<void(sss_patch_views)>& save_callback = &PatchDraper::default_callback);

#endif // PATCH_DRAPER_H
