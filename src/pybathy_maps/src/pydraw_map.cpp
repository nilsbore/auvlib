/* Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <bathy_maps/draw_map.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace std_data;

namespace py = pybind11;

PYBIND11_MODULE(draw_map, m) {
    m.doc() = "Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data"; // optional module docstring
    py::class_<BathyMapImage>(m, "BathyMapImage", "Class for constructing mesh from multibeam data")
        .def(py::init<const mbes_ping::PingsT&, int, int>(), "Constructor, takes mbes_ping::PingsT and height and width of height map")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::Matrix2d&>(), "Constructor, takes pre-computed height map matrix and bounds")
        .def("draw_track", (void (BathyMapImage::*)(const mbes_ping::PingsT&) ) &BathyMapImage::draw_track, "Draw vehicle track from mbes_ping::PingsT")
        .def("draw_track", (void (BathyMapImage::*)(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >&) ) &BathyMapImage::draw_track, "Draw vehicle track from list of Vector3d")
        .def("draw_height_map", (void (BathyMapImage::*)(const mbes_ping::PingsT&) ) &BathyMapImage::draw_height_map, "Draw height map from mbes_ping::PingsT")
        .def("draw_height_map", (void (BathyMapImage::*)(const Eigen::MatrixXd&) ) &BathyMapImage::draw_height_map, "Draw height map from pre-computed height map matrix")
        .def("draw_indices", &BathyMapImage::draw_indices, "Draw indices of the pings within the map, from mbes_ping::PingsT")
        .def("draw_back_scatter_map", &BathyMapImage::draw_back_scatter_map, "Draw back scatter map from mbes_ping::PingsT")
        .def("draw_targets", &BathyMapImage::draw_targets, "Draw point targets from dict of points")
        .def("draw_blue_pose", &BathyMapImage::draw_blue_pose, "Draw pose with a heading on top of map")
        .def("draw_red_pose", &BathyMapImage::draw_red_pose, "Draw pose with a heading on top of map")
        .def("rotate_crop_image", &BathyMapImage::rotate_crop_image, "Rotate and crop the image from a start and an end point, and a width in between")
        .def("show", &BathyMapImage::show, "Show the drawn bathy map")
        .def("blip", &BathyMapImage::blip, "Blip the drawn bathy map")
        .def("make_image", &BathyMapImage::make_image, "Get the drawn image")
        .def("write_image", &BathyMapImage::write_image_from_str, "Save image to file");

    // from http://alexsm.com/pybind11-buffer-protocol-opencv-to-numpy/
    pybind11::class_<cv::Mat>(m, "Image", pybind11::buffer_protocol())
        .def_buffer([](cv::Mat& im) -> pybind11::buffer_info {
            return pybind11::buffer_info(
                // Pointer to buffer
                im.data,
                // Size of one scalar
                sizeof(unsigned char),
                // Python struct-style format descriptor
                pybind11::format_descriptor<unsigned char>::format(),
                // Number of dimensions
                3,
                // Buffer dimensions
                { im.rows, im.cols, im.channels() },
                // Strides (in bytes) for each index
                {
                    sizeof(unsigned char) * im.channels() * im.cols,
                    sizeof(unsigned char) * im.channels(),
                    sizeof(unsigned char)
                }
            );
        });
}
