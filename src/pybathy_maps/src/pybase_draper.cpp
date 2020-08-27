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

#include <bathy_maps/base_draper.h>
#include <bathy_maps/view_draper.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace std_data;
using namespace csv_data;

namespace py = pybind11;

PYBIND11_MODULE(base_draper, m) {
    m.doc() = "Functions for draping a mesh with sidescan data"; // optional module docstring
    py::class_<ping_draping_result>(m, "ping_draping_result", "Class for representing the intermediate result from draping on sidescan ping side")
        .def(py::init<>())
        .def_readwrite("sensor_origin", &ping_draping_result::sensor_origin, "Origin of sensor when capturing ping")
        .def_readwrite("hits_points", &ping_draping_result::hits_points, "3D positions of hits, see hits_inds for corresponding sidescan index")
        .def_readwrite("hits_inds", &ping_draping_result::hits_inds, "Ping time index of the hits")
        .def_readwrite("hits_intensities", &ping_draping_result::hits_intensities, "Downsampledintensities from sidescan")
        .def_readwrite("time_bin_points", &ping_draping_result::time_bin_points, "Depths corresponding to the ping intensities")
        .def_readwrite("time_bin_normals", &ping_draping_result::time_bin_normals, "Normals corresponding to the ping intensities")
        .def_readwrite("time_bin_model_intensities", &ping_draping_result::time_bin_model_intensities, "Model intensities corresponding to the real intensities")
        .def_static("read_data", &read_data_from_str<ping_draping_result::ResultsT>, "Read ping_draping_result::ResultsT from .cereal file");

    py::class_<BaseDraper>(m, "BaseDraper", "Class for draping the whole data set of sidescan pings onto a bathymetry mesh")
        // Methods inherited from BaseDraper:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const BaseDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("project_ping", &BaseDraper::project_ping, "Project a ping onto the mesh and get intermediate draping results. Provide the desired downsampling of the ping as the second parameter")
        .def("set_sidescan_yaw", &BaseDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_sidescan_port_stbd_offsets", &BaseDraper::set_sidescan_port_stbd_offsets, "Set offsets of sidescan port and stbd sides with respect to nav frame")
        .def("set_tracing_map_size", &BaseDraper::set_tracing_map_size, "Set size of slice of map where we do ray tracing. Smaller makes it faster but you might cut off valid sidescan angles")
        .def("set_intensity_multiplier", &BaseDraper::set_intensity_multiplier, "Set a value to multiply the sidescan intensity with when displaying on top of mesh")
        .def("set_ray_tracing_enabled", &BaseDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences");

    py::class_<ViewDraper>(m, "ViewDraper", "Base class for draping sidescan pings onto a bathymetry mesh")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const std_data::sss_ping::PingsT&, const ViewDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("add_texture_intensities", &ViewDraper::add_texture_intensities, "Add the intensities of draping result hits and intensities")
        .def("set_rgb_texture", &ViewDraper::set_rgb_texture, "Set a new texture to color the mesh, each RGB channel provided separately")
        .def("get_texture_image", &ViewDraper::get_texture_image, "Get the texture image, defined within bounds, with resolution of 1m")
        .def("set_sidescan_yaw", &ViewDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_sidescan_port_stbd_offsets", &ViewDraper::set_sidescan_port_stbd_offsets, "Set offsets of sidescan port and stbd sides with respect to nav frame")
        .def("set_tracing_map_size", &ViewDraper::set_tracing_map_size, "Set size of slice of map where we do ray tracing. Smaller makes it faster but you might cut off valid sidescan angles")
        .def("set_intensity_multiplier", &ViewDraper::set_intensity_multiplier, "Set a value to multiply the sidescan intensity with when displaying on top of mesh")
        .def("set_ray_tracing_enabled", &ViewDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &ViewDraper::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("set_callback", &ViewDraper::set_callback, "Set the function to be called when one ping has been draped")
        .def("show", &ViewDraper::show, "Start the draping, and show the visualizer");

    m.def("compute_bin_intensities", &compute_bin_intensities, "Dowsample the intensities of a ping to a vector of a desired length");
    m.def("color_jet_from_mesh", &color_jet_from_mesh, "Get a jet color scheme from a vertex matrix");
    m.def("get_vehicle_mesh", &get_vehicle_mesh, "Get vertices, faces, and colors for vehicle");
    m.def("write_data", &write_data_from_str<ping_draping_result::ResultsT>, "Write ping_draping_result::ResultsT to .cereal file");
    m.def("write_data", &write_data_from_str<ping_draping_result>, "Write ping_draping_result to .cereal file");
    //m.def("write_data", &write_data_from_str<sss_draping_result>, "Write sss_draping_result to .cereal file");
    m.def("drape_viewer", &drape_viewer, "Draping only for visualization, with no data being produced");
}
