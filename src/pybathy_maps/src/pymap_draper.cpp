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

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace std_data;
using namespace xtf_data;
using namespace csv_data;

namespace py = pybind11;

PYBIND11_MODULE(map_draper, m) {
    m.doc() = "Functions for draping a mesh with sidescan data"; // optional module docstring
    py::class_<sss_map_image>(m, "sss_map_image", "Class for sidescan views of a patch from different survey lines")
        .def(py::init<>())
        .def_readwrite("bounds", &sss_map_image::bounds, "Member")
        .def_readwrite("sss_map_image", &sss_map_image::sss_map_image, "Member")
        .def_readwrite("sss_waterfall_image", &sss_map_image::sss_waterfall_image, "Member")
        .def_readwrite("sss_waterfall_cross_track", &sss_map_image::sss_waterfall_cross_track, "Member")
        .def_readwrite("sss_waterfall_depth", &sss_map_image::sss_waterfall_depth, "Member")
        .def_readwrite("pos", &sss_map_image::pos, "Member")
        .def_static("read_data", &read_data_from_str<sss_map_image::ImagesT>, "Read sss_map_image::ImagesT from .cereal file");


    py::class_<MapDraper>(m, "MapDraper", "Class for draping the whole data set of sidescan pings onto a bathymetry mesh")
        // Methods inherited from MapDraper:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const xtf_sss_ping::PingsT&, const MapDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &MapDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_ray_tracing_enabled", &MapDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &MapDraper::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &MapDraper::show, "Start the draping, and show the visualizer")
        // Methods unique to MapDraper:
        .def("set_resolution", &MapDraper::set_resolution, "Set the resolution of the gathered maps, default is ~3.75")
        .def("set_image_callback", &MapDraper::set_image_callback, "Set the function to be called when an entire sidescan map is done")
        .def("get_images", &MapDraper::get_images, "Get all the sss_map_image::ImagesT that have been gathered so far");

    m.def("drape_maps", &drape_maps, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_map_image::ViewsT");
    m.def("color_jet_from_mesh", &color_jet_from_mesh, "Get a jet color scheme from a vertex matrix");
    m.def("get_vehicle_mesh", &get_vehicle_mesh, "Get vertices, faces, and colors for vehicle");
    m.def("convert_maps_to_patches", &convert_maps_to_patches, "Convert sss_map_image::ImagesT to sss_patch_views::ViewsT");
    m.def("convert_maps_to_single_angle_patches", &convert_maps_to_single_angle_patches, "Convert sss_map_image::ImagesT to sss_patch_views::ViewsT, but only from one angle in each case");
    m.def("write_data", &write_data_from_str<sss_map_image::ImagesT>, "Write sss_map_image::ImagesT to .cereal file");
}
