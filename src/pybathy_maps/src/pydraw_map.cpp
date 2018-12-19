#include <bathy_maps/draw_map.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace std_data;

namespace py = pybind11;

PYBIND11_MODULE(draw_map, m) {
    m.doc() = "Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data"; // optional module docstring
    py::class_<BathyMapImage>(m, "BathyMapImage", "Class for constructing mesh from multibeam data")
        .def(py::init<mbes_ping::PingsT&, int, int>(), "Constructor, takes mbes_ping::PingsT and height and width of height map")
        .def("draw_track", (void (BathyMapImage::*)(mbes_ping::PingsT&) ) &BathyMapImage::draw_track, "Draw vehicle track from mbes_ping::PingsT")
        .def("draw_height_map", &BathyMapImage::draw_height_map, "Draw height map from mbes_ping::PingsT")
        .def("draw_back_scatter_map", &BathyMapImage::draw_back_scatter_map, "Draw back scatter map from mbes_ping::PingsT")
        .def("draw_targets", &BathyMapImage::draw_targets, "Draw point targets from dict of points")
        .def("show", &BathyMapImage::show, "Show the drawn bathy map")
        .def("write_image", &BathyMapImage::write_image_from_str, "Save image to file");
}
