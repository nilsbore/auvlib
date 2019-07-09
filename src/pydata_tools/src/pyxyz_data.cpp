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

#include <data_tools/xyz_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace std_data;

namespace py = pybind11;

PYBIND11_MODULE(xyz_data, m) {
    m.doc() = "Basic utilities for working with the xyz file format"; // optional module docstring

    py::class_<xyz_data::Points>(m, "cloud", "Class for xyz point cloud type")
        .def(py::init<>())
        .def_static("from_pings", &xyz_data::from_pings, "Create vector of xyz_data::Points from std_data::mbes_ping::PingsT by splitting at first_in_file_==True")
        .def_static("to_matrix", &xyz_data::to_matrix, "Create an Nx3 matrix from the list of points")
        .def_static("from_matrix", &xyz_data::from_matrix, "Create Points from an Nx3 matrix")
        .def_static("parse_file", &parse_file_from_str<Eigen::Vector3d>, "Parse xyz_data::Points from .xyz file")
        .def_static("parse_folder", &parse_folder_from_str<Eigen::Vector3d>, "Parse xyz_data::Points from folder of .xyz files")
        .def_static("read_data", &read_data_from_str<xyz_data::Points>, "Read xyz_data::Points from .cereal file");


    m.def("subsample_points", &xyz_data::subsample_points, "Subsample by skipping N points at a time");
    m.def("transform_points", &xyz_data::transform_points, "Transform using 4x4 transformation matrix T");
    m.def("write_data", &write_data_from_str<xyz_data::Points>, "Write array of Vector3d to .cereal file");
}
