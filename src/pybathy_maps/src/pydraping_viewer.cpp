#include <bathy_maps/draping_viewer.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(draping_viewer, m) {
    m.doc() = "Functions for draping a mesh with sidescan data"; // optional module docstring
    m.def("overlay_sss", &overlay_sss, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh");
}
