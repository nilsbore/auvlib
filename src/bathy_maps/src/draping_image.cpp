#include <bathy_maps/draping_image.h>

#include <igl/readSTL.h>
#include <igl/unproject_onto_mesh.h>
#include <bathy_maps/drape_mesh.h>

using namespace std;

draping_image::draping_image(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
    const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
    const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset,
    const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
    const BoundsT& bounds, double resolution, const std::function<void(sss_map_image)>& save_callback)
    : draping_generator(V1, F1, C1, V2, F2, C2, pings, offset, sound_speeds, sensor_yaw),
      bounds(bounds), resolution(resolution), save_callback(save_callback), map_image_builder(bounds, resolution)
{
    viewer.callback_pre_draw = std::bind(&draping_image::callback_pre_draw, this, std::placeholders::_1);
}

bool draping_image::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    // check if ping is first_in_file_, in that case, split off new image
    if ((i == pings.size() - 1 || (i+1 < pings.size() && pings[i+1].first_in_file_)) && !map_image_builder.empty()) {
        sss_map_image map_image = map_image_builder.finish();
        save_callback(map_image);
        map_images.push_back(map_image);
        map_image_builder = sss_map_image_builder(bounds, resolution);
    }

    if (i >= pings.size()) {
        return false;
    }

    Eigen::MatrixXd hits_left_intensities, hits_right_intensities;
    Eigen::Vector3d pos;
    tie(hits_left_intensities, hits_right_intensities, pos) = project_sss();

    map_image_builder.add_hits(hits_left_intensities, pos, true);
    map_image_builder.add_hits(hits_right_intensities, pos, false);

    ++i;

    return false;
}

sss_map_image::ImagesT draping_image::get_images()
{
    return map_images;
}

sss_map_image::ImagesT drape_images(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                  const draping_image::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                  const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw,
                  double resolution, const std::function<void(sss_map_image)>& save_callback)
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Cb;
    tie(Vb, Fb, Cb) = get_vehicle_mesh();

    Eigen::MatrixXd C_jet = color_jet_from_mesh(V);

    Eigen::Vector3d offset(bounds(0, 0), bounds(0, 1), 0.);
    draping_image viewer(V, F, C_jet, Vb, Fb, Cb, pings, offset, sound_speeds, sensor_yaw, bounds, resolution, save_callback);
    viewer.launch();

    return viewer.get_images();

    //sss_patch_views::ViewsT views = viewer.get_patch_views();

    //return views;
}
