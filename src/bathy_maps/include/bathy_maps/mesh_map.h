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

#ifndef MESH_MAP_H
#define MESH_MAP_H

#include <Eigen/Dense>
#include <data_tools/std_data.h>
#include <data_tools/xtf_data.h>

namespace mesh_map {

    using BoundsT = Eigen::Matrix2d;

    std::pair<Eigen::MatrixXd, Eigen::MatrixXi> mesh_from_height_map(const Eigen::MatrixXd& height_map, const BoundsT& bounds);
    std::pair<Eigen::MatrixXd, BoundsT> height_map_from_pings(const std_data::mbes_ping::PingsT& pings, double res);
    std::pair<Eigen::MatrixXd, BoundsT> height_map_from_cloud(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& cloud, double res);
    std::pair<Eigen::MatrixXd, BoundsT> height_map_from_dtm_cloud(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& cloud, double res);
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, BoundsT> mesh_from_pings(const std_data::mbes_ping::PingsT& pings, double res=0.5);
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, BoundsT> mesh_from_cloud(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& cloud, double res);
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, BoundsT> mesh_from_dtm_cloud(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& cloud, double res);
    void show_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    //void show_textured_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& height_map, const BoundsT& bounds);
    void show_height_map(const Eigen::MatrixXd& height_map);
    void show_textured_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                            const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
                            const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
                            const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B,
                            const BoundsT& bounds);

    std::tuple<Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>,
               Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>,
               Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> >
    height_map_to_texture(const Eigen::MatrixXd& height_map);

    std::pair<Eigen::MatrixXd, Eigen::MatrixXi> cut_square_around_point(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                                                        const Eigen::Vector2d& p, double side);

    // N should be the vertex normals, e.g. from compute_normals
    Eigen::Vector3d normal_at_point(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& N, const Eigen::Vector3d& origin);
    double depth_at_point(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::Vector3d& origin);
    Eigen::VectorXd depths_at_points(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& origins);
    Eigen::MatrixXd normals_at_points(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& N, const Eigen::MatrixXd& origins);
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, BoundsT> mesh_and_normals_from_pings(const std_data::mbes_ping::PingsT& pings, double res);
    Eigen::MatrixXd shade_image_from_normals(const Eigen::MatrixXd& N, const BoundsT& bounds, double res, const Eigen::Vector3d& light_dir);
    Eigen::MatrixXd compute_normals(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    Eigen::MatrixXd normals_at_grid_points(const Eigen::MatrixXd& points, const Eigen::MatrixXd& N, const BoundsT& bounds, double res);

    void write_dae_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const boost::filesystem::path& filename);
    void write_dae_mesh_from_str(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const std::string& filename);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXi> read_ply_mesh_from_str(const std::string& filename);

}

#endif // MESH_MAP_H
