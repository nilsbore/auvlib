#include <bathy_maps/draw_map.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(draw_map, m) {
    m.doc() = "Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data"; // optional module docstring
    py::class_<bathy_map_image>(m, "bathy_map_image", "Class for constructing mesh from multibeam data")
        .def(py::init<mbes_ping::PingsT&, int, int>())
        .def("draw_track", (void (bathy_map_image::*)(mbes_ping::PingsT&) ) &bathy_map_image::draw_track, "Draw vehicle track from mbes_ping::PingsT")
        .def("draw_height_map", &bathy_map_image::draw_height_map, "Draw height map from mbes_ping::PingsT")
        .def("draw_back_scatter_map", &bathy_map_image::draw_back_scatter_map, "Draw back scatter map from mbes_ping::PingsT")
        .def("draw_targets", &bathy_map_image::draw_targets, "Draw point targets from dict of points")
        .def("save_image", &bathy_map_image::save_image_from_str, "Save image to file");
}
