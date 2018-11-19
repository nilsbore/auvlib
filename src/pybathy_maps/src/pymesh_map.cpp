#include <bathy_maps/draw_map.h>
#include <bathy_maps/mesh_map.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(mesh_map, m) {
    m.doc() = "Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data"; // optional module docstring
    py::class_<bathy_map_mesh>(m, "bathy_map_mesh", "Class for constructing mesh from multibeam data")
        .def(py::init<>())
        .def("mesh_from_height_map", &bathy_map_mesh::mesh_from_height_map, "Construct mesh from height map")
        .def("height_map_from_pings", &bathy_map_mesh::height_map_from_pings, "Construct height map from mbes_ping::PingsT")
        .def("mesh_from_pings", &bathy_map_mesh::mesh_from_pings, "Construct mesh from mbes_ping::PingsT")
        .def("display_mesh", &bathy_map_mesh::display_mesh, "Display mesh using igl viewer")
        .def("overlay_sss", &bathy_map_mesh::overlay_sss, "Overlay xtf_sss_ping::PingsT sidescan data on the mesh")
        .def("display_height_map", &bathy_map_mesh::display_height_map, "Display height map using opencv");
}
