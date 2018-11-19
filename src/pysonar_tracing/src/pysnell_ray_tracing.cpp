#include <sonar_tracing/snell_ray_tracing.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(snell_ray_tracing, m) {
    m.doc() = "Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data"; // optional module docstring
    m.def("trace_multiple_layers", &trace_multiple_layers, "Trace multiple rays to a sequence of layers");
    m.def("trace_single_layers", &trace_single_layers, "Trace single ray through a sequence of layers");
    m.def("visualize_rays", &visualize_rays, "Visualize traces layers using opencv");
}
