#include <bathy_maps/draping_viewer.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(draping_viewer, m) {
    m.doc() = "Functions for draping a mesh with sidescan data"; // optional module docstring
    py::class_<sss_patch_views>(m, "sss_patch_views", "Class for sidescan views of a patch from different survey lines")
        .def(py::init<>())
        .def_readwrite("patch_height", &sss_patch_views::patch_height, "Member")
        .def_readwrite("sss_views", &sss_patch_views::sss_views, "Member")
        .def_readwrite("patch_view_pos", &sss_patch_views::patch_view_pos, "Member")
        .def_static("read_data", &read_data_from_str<sss_patch_views::ViewsT>, "Read sss_patch_views::ViewsT from .cereal file");

    m.def("overlay_sss", &overlay_sss, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT");
    m.def("write_data", &write_data_from_str<sss_patch_views::ViewsT>, "Write sss_patch_views::ViewsT to .cereal file");
}
