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
    m.def("height_map_from_cloud", &mesh_map::height_map_from_cloud, "Construct height map from vector<Eigen::Vector3d>");
    m.def("height_map_from_dtm_cloud", &mesh_map::height_map_from_dtm_cloud, "Construct height map from vector<Eigen::Vector3d>");
    m.def("mesh_from_pings", &mesh_map::mesh_from_pings, "Construct mesh from mbes_ping::PingsT");
    m.def("mesh_from_cloud", &mesh_map::mesh_from_cloud, "Construct mesh from vector<Eigen::Vector3d>");
    m.def("mesh_from_dtm_cloud", &mesh_map::mesh_from_dtm_cloud, "Construct mesh from vector<Eigen::Vector3d>");
    m.def("show_mesh", &mesh_map::show_mesh, "Display mesh using igl viewer");
    m.def("show_textured_mesh", &mesh_map::show_textured_mesh, "Display textured mesh using igl viewer");
    m.def("show_height_map", &mesh_map::show_height_map, "Display height map using opencv");
    m.def("height_map_to_texture", &mesh_map::height_map_to_texture, "Get R, G, B color textures from height map");
    m.def("write_dae_mesh", &mesh_map::write_dae_mesh_from_str, "Write vertices and faces to a .dae file");
    m.def("read_ply_mesh", &mesh_map::read_ply_mesh_from_str, "Read vertices and faces from a .ply file");
    m.def("cut_square_around_point", &mesh_map::cut_square_around_point, "Cut out a square from within the mesh, around point p with side length");
    m.def("normal_at_point", &mesh_map::normal_at_point, "Get the normal at point in the mesh, from vertex normals, returns (0, 0 -1) if no mesh");
    m.def("depth_at_point", &mesh_map::depth_at_point, "Get the depth at point in the mesh, returns 0 if no mesh at point");
    m.def("normals_at_points", &mesh_map::normals_at_points, "Get the normals at points in the mesh, from vertex normals, returns (0, 0 -1) if no mesh");
    m.def("depths_at_points", &mesh_map::depths_at_points, "Get the depths at points in the mesh, returns 0 if no mesh at point");
    m.def("mesh_and_normals_from_pings", &mesh_map::mesh_and_normals_from_pings, "Get vertices, faces, normals and bounds form mbes_ping::PingsT");
    m.def("shade_image_from_normals", &mesh_map::shade_image_from_normals, "Compute [0, 1] shade image from normals and lighting direction");
    m.def("compute_normals", &mesh_map::compute_normals, "Compute normals from the mesh, per vertex");
    m.def("normals_at_grid_points", &mesh_map::normals_at_grid_points, "Get the normals at a set of points on the mesh");

}
