#include <bathy_maps/draping_viewer.h>
#include <bathy_maps/draping_generator.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace py = pybind11;

PYBIND11_MODULE(draping_viewer, m) {
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

    py::class_<draping_generator>(m, "draping_generator", "Base class for draping sidescan pings onto a bathymetry mesh")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const xtf_sss_ping::PingsT&, const draping_generator::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &draping_generator::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_ray_tracing_enabled", &draping_generator::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &draping_generator::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &draping_generator::show, "Start the draping, and show the visualizer");

    py::class_<draping_patches>(m, "draping_patches", "Base class for draping sidescan pings onto a particular point of a bathymetry mesh")
        // Methods inherited from draping_generator:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const xtf_sss_ping::PingsT&, const draping_patches::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &draping_patches::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_ray_tracing_enabled", &draping_patches::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &draping_patches::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &draping_patches::show, "Start the draping, and show the visualizer")
        // Methods unique to draping_patches:
        .def("set_patch_callback", &draping_patches::set_patch_callback, "Set the function to be called when all views of a patch have been assembled")
        .def("get_patch_views", &draping_patches::get_patch_views, "Get all the sss_patch_views::PatchesT that have been gathered so far");

    m.def("overlay_sss", &overlay_sss, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT");
    m.def("color_jet_from_mesh", &color_jet_from_mesh, "Get a jet color scheme from a vertex matrix");
    m.def("get_vehicle_mesh", &get_vehicle_mesh, "Get vertices, faces, and colors for vehicle");
    m.def("generate_draping", &generate_draping, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT");
    m.def("write_data", &write_data_from_str<sss_patch_views::ViewsT>, "Write sss_patch_views::ViewsT to .cereal file");
}
