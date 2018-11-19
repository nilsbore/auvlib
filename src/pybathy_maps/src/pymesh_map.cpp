#include <bathy_maps/draw_map.h>
#include <bathy_maps/mesh_map.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(mesh_map, m) {
    m.doc() = "Data structure for constructing and viewing a bathymetry mesh"; // optional module docstring
    py::class_<bathy_map_mesh>(m, "bathy_map_mesh")
        .def(py::init<>())
        .def("mesh_from_height_map", &bathy_map_mesh::mesh_from_height_map)
        .def("height_map_from_pings", &bathy_map_mesh::height_map_from_pings)
        .def("mesh_from_pings", &bathy_map_mesh::mesh_from_pings)
        .def("display_mesh", &bathy_map_mesh::display_mesh)
        .def("overlay_sss", &bathy_map_mesh::overlay_sss)
        .def("display_height_map", &bathy_map_mesh::display_height_map);
}
