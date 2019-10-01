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

#include <bathy_maps/align_map.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

struct BBTree {

    igl::AABB<Eigen::MatrixXd, 3> tree;

    BBTree(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) : tree()
    {
        tree.init(V, F);
    }

};

PYBIND11_MODULE(align_map, m) {
    m.doc() = "Functions for aligning individual point clouds to a bathymetry mesh"; // optional module docstring

    py::class_<BBTree>(m, "BBTree", "Class for constructing AABB bounding box tree from mesh")
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&>(), "Constructor")
        .def_readonly("tree", &BBTree::tree, "Tree data structure used for speeding up proximity calculation");
    py::class_<igl::AABB<Eigen::MatrixXd, 3> >(m, "iglAABB", "Class for constructing AABB bounding box tree from mesh");
    /*py::class_<igl::AABB<Eigen::MatrixXd, 3> >(m, "BBTree", "Class for constructing AABB bounding box tree from mesh")
        .def(py::init<>(), "Constructor")
        .def("init", (void(igl::AABB<Eigen::MatrixXd, 3>::*)(const Eigen::MatrixXd&, const Eigen::MatrixXi&)) &igl::AABB<Eigen::MatrixXd, 3>::init<Eigen::MatrixXd>, "Draw vehicle track from mbes_ping::PingsT");*/

    m.def("points_to_mesh_rmse", (double(*)(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXi&)) &align_map::points_to_mesh_rmse, "Compute points RMSE wrt mesh");
    m.def("points_to_mesh_rmse", (double(*)(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXi&, const igl::AABB<Eigen::MatrixXd, 3>&)) &align_map::points_to_mesh_rmse, "Compute points RMSE wrt mesh");
    m.def("align_maps_icp", &align_map::align_maps_icp, "Align several maps into one mesh");
    m.def("align_points_to_mesh_icp", (std::pair<Eigen::Matrix4d, bool>(*)(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXi&, const igl::AABB<Eigen::MatrixXd, 3>&)) &align_map::align_points_to_mesh_icp, "Register points to a mesh with ICP");
    m.def("align_points_to_mesh_icp_vis", &align_map::align_points_to_mesh_icp_vis, "Register points to a mesh with ICP");
    m.def("filter_points_mesh_offset", &align_map::filter_points_mesh_offset, "Filter points too far away from the mesh");
    m.def("compute_overlap_ratio", &align_map::compute_overlap_ratio, "Compute ratio of points in one cloud that is close to the other");
    m.def("show_multiple_clouds", &align_map::show_multiple_clouds, "Show multiple clouds in different colors");

}
