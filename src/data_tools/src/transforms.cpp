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

#include <data_tools/transforms.h>

namespace data_transforms {

Eigen::Matrix3d euler_to_matrix(double x, double y, double z)
{
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).matrix();
    return Rx*Ry*Rz;
}

std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > euler_to_matrices(double x, double y, double z) 
{
    Eigen::Matrix3d Rx = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).matrix();
    return {Rx, Ry, Rz};
}

std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > euler_to_diff_matrices(double x, double y, double z)
{
    //Eigen::Matrix3d Rx; Rx << 1., 0., 0., 0., cos(x), -sin(x), 0., sin(x), cos(x);
    Eigen::Matrix3d Rx; Rx << 0., 0., 0., 0., -sin(x), -cos(x), 0., cos(x), -sin(x);
    //Eigen::Matrix3d Ry; Ry << cos(x), 0., sin(x), 0., 1., 0., -sin(x), 0., cos(x);
    Eigen::Matrix3d Ry; Ry << -sin(x), 0., cos(x), 0., 0., 0., -cos(x), 0., -sin(x);
    //Eigen::Matrix3d Rz; Rz << cos(x), -sin(x), 0., sin(x), cos(x), 0., 0., 0., 1.;
    Eigen::Matrix3d Rz; Rz << -sin(x), -cos(x), 0., cos(x), -sin(x), 0., 0., 0., 0.;
    return {Rx, Ry, Rz};
}

}
