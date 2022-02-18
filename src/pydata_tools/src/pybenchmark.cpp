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

#include <data_tools/benchmark.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace benchmark;

PYBIND11_MODULE(benchmark, m) {
    m.doc() = "Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data"; // optional module docstring

    py::class_<track_error_benchmark>(m, "track_error_benchmark", "Class for constructing a multibeam alignment benchmark")
        .def(py::init<const std::string&>(), "Constructor, takes the dataset name")
        .def(py::init<>(), "Constructor, default dataset name 'default'")
        .def("add_ground_truth", (void (track_error_benchmark::*)(std_data::mbes_ping::PingsT&) ) &track_error_benchmark::add_ground_truth, "Add the ground truth dataset, need to call this before add_benchmark")
        .def("add_ground_truth", (void (track_error_benchmark::*)(std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >&, std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >& ) ) &track_error_benchmark::add_ground_truth, "Add the ground truth dataset, need to call this before add_benchmark")
        .def("add_benchmark", (void (track_error_benchmark::*)(std_data::mbes_ping::PingsT&, const std::string&) ) &track_error_benchmark::add_benchmark, "Add a benchmark from std_data::mbes_ping::PingsT")
        .def("add_benchmark", (void (track_error_benchmark::*)(std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >&, std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >&, const std::string&) ) &track_error_benchmark::add_benchmark, "Add a benchmark from PointsT")
        .def("print_summary", (void (track_error_benchmark::*)() ) &track_error_benchmark::print_summary, "Prints summary of benchmarking results");

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
