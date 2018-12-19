#include <bathy_maps/patch_draper.h>
#include <bathy_maps/base_draper.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace data_structures;
using namespace xtf_data;
using namespace csv_data;

namespace py = pybind11;

PYBIND11_MODULE(patch_draper, m) {
    m.doc() = "Functions for draping a mesh with sidescan data"; // optional module docstring
    py::class_<sss_patch_views>(m, "sss_patch_views", "Class for sidescan views of a patch from different survey lines")
        .def(py::init<>())
        .def_readwrite("patch_size", &sss_patch_views::patch_size, "Member")
        .def_readwrite("patch_origin", &sss_patch_views::patch_origin, "Member")
        .def_readwrite("patch_height", &sss_patch_views::patch_height, "Member")
        .def_readwrite("sss_views", &sss_patch_views::sss_views, "Member")
        .def_readwrite("patch_view_pos", &sss_patch_views::patch_view_pos, "Member")
        .def_readwrite("patch_view_dirs", &sss_patch_views::patch_view_dirs, "Member")
        .def_static("read_data", &read_data_from_str<sss_patch_views::ViewsT>, "Read sss_patch_views::ViewsT from .cereal file");

    py::class_<BaseDraper>(m, "BaseDraper", "Base class for draping sidescan pings onto a bathymetry mesh")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const xtf_sss_ping::PingsT&, const BaseDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &BaseDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_ray_tracing_enabled", &BaseDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &BaseDraper::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &BaseDraper::show, "Start the draping, and show the visualizer");

    py::class_<PatchDraper>(m, "PatchDraper", "Base class for draping sidescan pings onto a particular point of a bathymetry mesh")
        // Methods inherited from draping_generator:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const xtf_sss_ping::PingsT&, const PatchDraper::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &PatchDraper::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_ray_tracing_enabled", &PatchDraper::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &PatchDraper::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &PatchDraper::show, "Start the draping, and show the visualizer")
        // Methods unique to PatchDraper:
        .def("set_patch_callback", &PatchDraper::set_patch_callback, "Set the function to be called when all views of a patch have been assembled")
        .def("get_patch_views", &PatchDraper::get_patch_views, "Get all the sss_patch_views::PatchesT that have been gathered so far");

    m.def("drape_patches", &drape_patches, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT");
    m.def("color_jet_from_mesh", &color_jet_from_mesh, "Get a jet color scheme from a vertex matrix");
    m.def("get_vehicle_mesh", &get_vehicle_mesh, "Get vertices, faces, and colors for vehicle");
    m.def("drape_viewer", &drape_viewer, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT");
    m.def("write_data", &write_data_from_str<sss_patch_views::ViewsT>, "Write sss_patch_views::ViewsT to .cereal file");
}
