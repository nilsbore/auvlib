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
#include <bathy_maps/base_draper.h>
#include <bathy_maps/sss_meas_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace std_data;
using namespace csv_data;

namespace py = pybind11;

using MapImageDraper = MapDraper<sss_map_image_builder>;
using MeasDataDraper = MapDraper<sss_meas_data_builder>;

PYBIND11_MODULE(map_draper, m) {
    m.doc() = "Functions for draping a mesh with sidescan data"; // optional module docstring
    py::class_<ping_draping_result>(m, "ping_draping_result", "Class for representing the intermediate result from draping on sidescan ping side")
        .def(py::init<>())
        .def_readwrite("sensor_origin", &ping_draping_result::sensor_origin, "Origin of sensor when capturing ping")
        .def_readwrite("hits_points", &ping_draping_result::hits_points, "3D positions of hits, see hits_inds for corresponding sidescan index")
        .def_readwrite("hits_inds", &ping_draping_result::hits_inds, "Ping time index of the hits")
        .def_readwrite("hits_intensities", &ping_draping_result::hits_intensities, "Downsampledintensities from sidescan")
        .def_readwrite("time_bin_points", &ping_draping_result::time_bin_points, "Depths corresponding to the ping intensities")
        .def_readwrite("time_bin_normals", &ping_draping_result::time_bin_normals, "Normals corresponding to the ping intensities")
        .def_readwrite("time_bin_model_intensities", &ping_draping_result::time_bin_model_intensities, "Model intensities corresponding to the real intensities");

    py::class_<sss_map_image>(m, "sss_map_image", "Class for sidescan views of a patch from different survey lines")
        .def(py::init<>())
        .def_readwrite("bounds", &sss_map_image::bounds, "Bounds of the mesh ([[minx, miny], [maxx, maxy]])")
        .def_readwrite("sss_map_image", &sss_map_image::sss_map_image_, "Intensities in map coordinates")
        .def_readwrite("sss_waterfall_image", &sss_map_image::sss_waterfall_image, "Sidescan waterfall image intensities")
        .def_readwrite("sss_waterfall_cross_track", &sss_map_image::sss_waterfall_cross_track, "Sidescan watefall cross track distances (not used)")
        .def_readwrite("sss_waterfall_depth", &sss_map_image::sss_waterfall_depth, "Sidescan waterfall hit depths")
        .def_readwrite("sss_waterfall_model", &sss_map_image::sss_waterfall_model, "Sidescan waterfall idealized simulated model")
        .def_readwrite("pos", &sss_map_image::pos, "Positions of pings")
        .def_readwrite("sss_ping_duration", &sss_map_image::sss_ping_duration, "Maximum ping time duration")
        .def_static("read_single", &read_data_from_str<sss_map_image>, "Read single sss_map_image from .cereal file")
        .def_static("read_data", &read_data_from_str<sss_map_image::ImagesT>, "Read sss_map_image::ImagesT from .cereal file");

    py::class_<sss_meas_data>(m, "sss_meas_data", "Class for sidescan views of a patch from different survey lines")
        .def(py::init<>())
        .def_readwrite("sss_waterfall_image", &sss_meas_data::sss_waterfall_image, "Sidescan waterfall image intensities")
        .def_readwrite("sss_waterfall_hits_X", &sss_meas_data::sss_waterfall_hits_X, "Sidescan waterfall hit x coordinates in map")
        .def_readwrite("sss_waterfall_hits_Y", &sss_meas_data::sss_waterfall_hits_Y, "Sidescan waterfall hit y coordinates in map")
        .def_readwrite("sss_waterfall_hits_Z", &sss_meas_data::sss_waterfall_hits_Z, "Sidescan waterfall hit z coordinates in map")
        .def_readwrite("ping_id", &sss_meas_data::ping_id, "Sequential ping IDs")
        .def_readwrite("pos", &sss_meas_data::pos, "Ping ENU positions")
        .def_readwrite("rpy", &sss_meas_data::rpy, "Ping attitudes (roll, pitch, yaw)")
        .def_static("read_single", &read_data_from_str<sss_meas_data>, "Read single sss_meas_data from .cereal file")
        .def_static("read_data", &read_data_from_str<sss_meas_data::ImagesT>, "Read sss_meas_data::ImagesT from .cereal file");

    py::class_<BaseDraper>(m, "BaseDraper", "Class for draping the whole data set of sidescan pings onto a bathymetry mesh")
        // Methods inherited from BaseDraper:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const BaseDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("project_ping", &BaseDraper::project_ping, "Project a ping onto the mesh and get intermediate draping results. Provide the desired downsampling of the ping as the second parameter")
        .def("compute_bin_intensities", &BaseDraper::compute_bin_intensities, "Dowsample the intensities of a ping to a vector of a desired length")
        .def("set_sidescan_yaw", &BaseDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_sidescan_port_stbd_offsets", &BaseDraper::set_sidescan_port_stbd_offsets, "Set offsets of sidescan port and stbd sides with respect to nav frame")
        .def("set_tracing_map_size", &BaseDraper::set_tracing_map_size, "Set size of slice of map where we do ray tracing. Smaller makes it faster but you might cut off valid sidescan angles")
        .def("set_intensity_multiplier", &BaseDraper::set_intensity_multiplier, "Set a value to multiply the sidescan intensity with when displaying on top of mesh")
        .def("set_ray_tracing_enabled", &BaseDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences");
        // Methods unique to BaseDraper:

    py::class_<MapImageDraper>(m, "MapDraper", "Class for draping the whole data set of sidescan pings onto a bathymetry mesh")
        // Methods inherited from MapImageDraper:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const std_data::sss_ping::PingsT&, const MapImageDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &MapImageDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_sidescan_port_stbd_offsets", &MapImageDraper::set_sidescan_port_stbd_offsets, "Set offsets of sidescan port and stbd sides with respect to nav frame")
        .def("set_tracing_map_size", &MapImageDraper::set_tracing_map_size, "Set size of slice of map where we do ray tracing. Smaller makes it faster but you might cut off valid sidescan angles")
        .def("set_intensity_multiplier", &MapImageDraper::set_intensity_multiplier, "Set a value to multiply the sidescan intensity with when displaying on top of mesh")
        .def("set_ray_tracing_enabled", &MapImageDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &MapImageDraper::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &MapImageDraper::show, "Start the draping, and show the visualizer")
        // Methods unique to MapImageDraper:
        .def("set_resolution", &MapImageDraper::set_resolution, "Set the resolution of the gathered maps, default is ~3.75")
        .def("set_image_callback", &MapImageDraper::set_image_callback, "Set the function to be called when an entire sidescan map is done")
        .def("set_store_map_images", &MapImageDraper::set_store_map_images, "Set if the draper should save and return map images at the end")
        .def("set_close_when_done", &MapImageDraper::set_close_when_done, "Set if the draper should close when done draping")
        .def("get_images", &MapImageDraper::get_images, "Get all the sss_map_image::ImagesT that have been gathered so far");

    py::class_<MeasDataDraper>(m, "MeasDataDraper", "Class for draping the whole data set of sidescan pings onto a bathymetry mesh")
        // Methods inherited from MeasDataDraper:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const std_data::sss_ping::PingsT&, const MeasDataDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &MeasDataDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_sidescan_port_stbd_offsets", &MeasDataDraper::set_sidescan_port_stbd_offsets, "Set offsets of sidescan port and stbd sides with respect to nav frame")
        .def("set_tracing_map_size", &MeasDataDraper::set_tracing_map_size, "Set size of slice of map where we do ray tracing. Smaller makes it faster but you might cut off valid sidescan angles")
        .def("set_intensity_multiplier", &MeasDataDraper::set_intensity_multiplier, "Set a value to multiply the sidescan intensity with when displaying on top of mesh")
        .def("set_ray_tracing_enabled", &MeasDataDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &MeasDataDraper::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &MeasDataDraper::show, "Start the draping, and show the visualizer")
        // Methods unique to MeasDataDraper:
        .def("set_resolution", &MeasDataDraper::set_resolution, "Set the resolution of the gathered maps, default is ~3.75")
        .def("set_image_callback", &MeasDataDraper::set_image_callback, "Set the function to be called when an entire sidescan map is done")
        .def("set_store_map_images", &MeasDataDraper::set_store_map_images, "Set if the draper should save and return map images at the end")
        .def("set_close_when_done", &MeasDataDraper::set_close_when_done, "Set if the draper should close when done draping")
        .def("get_images", &MeasDataDraper::get_images, "Get all the sss_map_image::ImagesT that have been gathered so far");

    m.def("drape_maps", &drape_maps, "Overlay sss_ping::PingsT sidescan data on a mesh and get sss_map_image::ViewsT");
    m.def("color_jet_from_mesh", &color_jet_from_mesh, "Get a jet color scheme from a vertex matrix");
    m.def("get_vehicle_mesh", &get_vehicle_mesh, "Get vertices, faces, and colors for vehicle");
    m.def("convert_maps_to_patches", &convert_maps_to_patches, "Convert sss_map_image::ImagesT to sss_patch_views::ViewsT");
    m.def("convert_maps_to_single_angle_patches", &convert_maps_to_single_angle_patches, "Convert sss_map_image::ImagesT to sss_patch_views::ViewsT, but only from one angle in each case");
    m.def("write_data", &write_data_from_str<sss_map_image::ImagesT>, "Write sss_map_image::ImagesT to .cereal file");
    m.def("write_data", &write_data_from_str<sss_map_image>, "Write sss_map_image to .cereal file");
    m.def("write_data", &write_data_from_str<sss_meas_data::ImagesT>, "Write sss_meas_data::ImagesT to .cereal file");
    m.def("write_data", &write_data_from_str<sss_meas_data>, "Write sss_meas_data to .cereal file");
}
