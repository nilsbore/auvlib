#include <bathy_maps/draping_image.h>
#include <bathy_maps/draping_generator.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace py = pybind11;

PYBIND11_MODULE(draping_image, m) {
    m.doc() = "Functions for draping a mesh with sidescan data"; // optional module docstring
    py::class_<sss_map_image>(m, "sss_map_image", "Class for sidescan views of a patch from different survey lines")
        .def(py::init<>())
        .def_readwrite("bounds", &sss_map_image::bounds, "Member")
        .def_readwrite("sss_map_image", &sss_map_image::sss_map_image, "Member")
        .def_readwrite("sss_waterfall_image", &sss_map_image::sss_waterfall_image, "Member")
        .def_readwrite("sss_waterfall_cross_track", &sss_map_image::sss_waterfall_cross_track, "Member")
        .def_readwrite("sss_waterfall_depth", &sss_map_image::sss_waterfall_depth, "Member")
        .def_readwrite("pos", &sss_map_image::pos, "Member")
        .def_static("read_data", &read_data_from_str<sss_map_image::ImagesT>, "Read sss_map_image::ImagesT from .cereal file");

    m.def("drape_images", &drape_images, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_map_image::ViewsT");
    m.def("color_jet_from_mesh", &color_jet_from_mesh, "Get a jet color scheme from a vertex matrix");
    m.def("get_vehicle_mesh", &get_vehicle_mesh, "Get vertices, faces, and colors for vehicle");
    m.def("convert_maps_to_patches", &convert_maps_to_patches, "Convert sss_map_image::ImagesT to sss_patch_views::ViewsT");
    m.def("write_data", &write_data_from_str<sss_map_image::ImagesT>, "Write sss_map_image::ImagesT to .cereal file");
}
