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


    py::class_<draping_image>(m, "draping_image", "Class for draping the whole data set of sidescan pings onto a bathymetry mesh")
        // Methods inherited from draping_image:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const xtf_sss_ping::PingsT&, const draping_image::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&>())
        .def("set_sidescan_yaw", &draping_image::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_ray_tracing_enabled", &draping_image::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &draping_image::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &draping_image::show, "Start the draping, and show the visualizer")
        // Methods unique to draping_image:
        .def("set_resolution", &draping_image::set_resolution, "Set the resolution of the gathered maps, default is ~3.75")
        .def("set_image_callback", &draping_image::set_image_callback, "Set the function to be called when an entire sidescan map is done")
        .def("get_images", &draping_image::get_images, "Get all the sss_map_image::ImagesT that have been gathered so far");

    m.def("drape_images", &drape_images, "Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_map_image::ViewsT");
    m.def("color_jet_from_mesh", &color_jet_from_mesh, "Get a jet color scheme from a vertex matrix");
    m.def("get_vehicle_mesh", &get_vehicle_mesh, "Get vertices, faces, and colors for vehicle");
    m.def("convert_maps_to_patches", &convert_maps_to_patches, "Convert sss_map_image::ImagesT to sss_patch_views::ViewsT");
    m.def("write_data", &write_data_from_str<sss_map_image::ImagesT>, "Write sss_map_image::ImagesT to .cereal file");
}
