#ifndef VIEW_DRAPER_H
#define VIEW_DRAPER_H

#include <bathy_maps/base_draper.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>

struct ViewDraper : public BaseDraper {
public:

    using BoundsT = Eigen::Matrix2d;

protected:

    igl::opengl::glfw::Viewer viewer; // ligigl viewer object
    std_data::sss_ping::PingsT pings; // sidescan pings used for draping
    int i; // timestep counter, one step per ping
    int nbr_time_bins;
    
    std::function<void(ping_draping_result, ping_draping_result)> save_callback;

    Eigen::MatrixXd texture_image; // used for displaying texture and checking coverage

    Eigen::MatrixXd V2; // vehicle mesh faces
    Eigen::MatrixXi F2; // vehicle mesh vertices
    Eigen::MatrixXd V; // combined bathy+vehicle, vis mesh vertices
    Eigen::MatrixXi F; // combined bathy+vehicle, vis mesh faces
    Eigen::MatrixXd C; // combined bathy+vehicle, vis mesh colors

    //void visualize_rays(const Eigen::MatrixXd& hits_left, const Eigen::MatrixXd& hits_right);
    void visualize_rays(const Eigen::Vector3d& sensor_origin, const Eigen::MatrixXd& hits, bool clear=false);

    void visualize_vehicle();

    void visualize_intensities();

    void set_texture(const Eigen::MatrixXd& texture, const BoundsT& texture_bounds);

    bool fast_is_mesh_underneath_vehicle(const Eigen::Vector3d& origin);

public:

    void set_rgb_texture(const Eigen::MatrixXd& R, const Eigen::MatrixXd& G, const Eigen::MatrixXd& B, const BoundsT& texture_bounds);
    void set_callback(const std::function<void(ping_draping_result, ping_draping_result)>& callback, int nbr_bins) { save_callback = callback; nbr_time_bins = nbr_bins; }
    Eigen::MatrixXd get_texture_image() { return texture_image; }
    void add_texture_intensities(const Eigen::MatrixXd& hits, const Eigen::VectorXd& intensities);
    void set_vehicle_mesh(const Eigen::MatrixXd& new_V2, const Eigen::MatrixXi& new_F2, const Eigen::MatrixXd& new_C2);

    ViewDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
               const std_data::sss_ping::PingsT& pings,
               const BoundsT& bounds,
               const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds = csv_data::csv_asvp_sound_speed::EntriesT());

    void show();
    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
};

std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd> get_vehicle_mesh();
Eigen::MatrixXd color_jet_from_mesh(const Eigen::MatrixXd& V);
void drape_viewer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                  const ViewDraper::BoundsT& bounds, const std_data::sss_ping::PingsT& pings,
                  const csv_data::csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw);

#endif // VIEW_DRAPER_H
