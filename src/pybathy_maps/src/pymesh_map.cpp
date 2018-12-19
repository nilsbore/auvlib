#include <bathy_maps/draw_map.h>
#include <bathy_maps/mesh_map.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(mesh_map, m) {
    m.doc() = "Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data"; // optional module docstring
    //py::class_<bathy_map_mesh>(m, "bathy_map_mesh", "Class for constructing mesh from multibeam data")
    //.def(py::init<>(), "Constructor")
    m.def("mesh_from_height_map", &mesh_map::mesh_from_height_map, "Construct mesh from height map");
    m.def("height_map_from_pings", &mesh_map::height_map_from_pings, "Construct height map from mbes_ping::PingsT");
    m.def("mesh_from_pings", &mesh_map::mesh_from_pings, "Construct mesh from mbes_ping::PingsT");
    m.def("show_mesh", &mesh_map::show_mesh, "Display mesh using igl viewer");
    m.def("show_height_map", &mesh_map::show_height_map, "Display height map using opencv");
}
