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
#include <bathy_maps/base_draper.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace std_data;
using namespace csv_data;

namespace py = pybind11;

PYBIND11_MODULE(patch_draper, m) {
    m.doc() = "Functions for draping a mesh with sidescan data"; // optional module docstring
    py::class_<sss_patch_views>(m, "sss_patch_views", "Class for sidescan views of a patch from different survey lines")
        .def(py::init<>())
        .def_readwrite("patch_size", &sss_patch_views::patch_size, "Size of the patch")
        .def_readwrite("patch_origin", &sss_patch_views::patch_origin, "Origin of patch in map coordinates")
        .def_readwrite("patch_height", &sss_patch_views::patch_height, "TODO")
        .def_readwrite("sss_views", &sss_patch_views::sss_views, "Sidescan views of patch")
        .def_readwrite("patch_view_pos", &sss_patch_views::patch_view_pos, "Positions of views")
        .def_readwrite("patch_view_dirs", &sss_patch_views::patch_view_dirs, "Directions of views (yaw)")
        .def_static("read_data", &read_data_from_str<sss_patch_views::ViewsT>, "Read sss_patch_views::ViewsT from .cereal file");

    py::class_<ViewDraper>(m, "ViewDraper", "Base class for draping sidescan pings onto a bathymetry mesh")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const std_data::sss_ping::PingsT&, const ViewDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("add_texture_intensities", &ViewDraper::add_texture_intensities, "Add the intensities of draping result hits and intensities")
        .def("get_texture_image", &ViewDraper::get_texture_image, "Get the texture image, defined within bounds, with resolution of 1m")
        .def("set_sidescan_yaw", &ViewDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_sidescan_port_stbd_offsets", &ViewDraper::set_sidescan_port_stbd_offsets, "Set offsets of sidescan port and stbd sides with respect to nav frame")
        .def("set_tracing_map_size", &ViewDraper::set_tracing_map_size, "Set size of slice of map where we do ray tracing. Smaller makes it faster but you might cut off valid sidescan angles")
        .def("set_intensity_multiplier", &ViewDraper::set_intensity_multiplier, "Set a value to multiply the sidescan intensity with when displaying on top of mesh")
        .def("set_ray_tracing_enabled", &ViewDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &ViewDraper::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &ViewDraper::show, "Start the draping, and show the visualizer");

    py::class_<PatchDraper>(m, "PatchDraper", "Base class for draping sidescan pings onto a particular point of a bathymetry mesh")
        // Methods inherited from draping_generator:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const std_data::sss_ping::PingsT&, const PatchDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &PatchDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_sidescan_port_stbd_offsets", &PatchDraper::set_sidescan_port_stbd_offsets, "Set offsets of sidescan port and stbd sides with respect to nav frame")
        .def("set_tracing_map_size", &PatchDraper::set_tracing_map_size, "Set size of slice of map where we do ray tracing. Smaller makes it faster but you might cut off valid sidescan angles")
        .def("set_ray_tracing_enabled", &PatchDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &PatchDraper::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &PatchDraper::show, "Start the draping, and show the visualizer")
        // Methods unique to PatchDraper:
        .def("set_patch_callback", &PatchDraper::set_patch_callback, "Set the function to be called when all views of a patch have been assembled")
        .def("get_patch_views", &PatchDraper::get_patch_views, "Get all the sss_patch_views::PatchesT that have been gathered so far");

    m.def("drape_patches", &drape_patches, "Overlay std_data::sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT");
    m.def("color_jet_from_mesh", &color_jet_from_mesh, "Get a jet color scheme from a vertex matrix");
    m.def("get_vehicle_mesh", &get_vehicle_mesh, "Get vertices, faces, and colors for vehicle");
    m.def("drape_viewer", &drape_viewer, "Overlay std_data::sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT");
    m.def("write_data", &write_data_from_str<sss_patch_views::ViewsT>, "Write sss_patch_views::ViewsT to .cereal file");
}
