#include <bathy_maps/map_draper.h>

#include <igl/readSTL.h>
#include <igl/unproject_onto_mesh.h>
#include <bathy_maps/drape_mesh.h>

using namespace std;

MapDraper::MapDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                     const xtf_sss_ping::PingsT& pings,
                     const BoundsT& bounds,
                     const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : BaseDraper(V1, F1, pings, bounds, sound_speeds),
      bounds(bounds), resolution(30./8.), save_callback(&default_callback),
      map_image_builder(bounds, 30./8., pings[0].port.pings.size())
{
    viewer.callback_pre_draw = std::bind(&MapDraper::callback_pre_draw, this, std::placeholders::_1);
}

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

void MapDraper::set_resolution(double new_resolution)
{ 
    resolution = new_resolution;
    map_image_builder = sss_map_image_builder(bounds, resolution, pings[i].port.pings.size());
}

sss_map_image::ImagesT MapDraper::get_images()
{
    return map_images;
}

sss_map_image::ImagesT drape_images(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
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
