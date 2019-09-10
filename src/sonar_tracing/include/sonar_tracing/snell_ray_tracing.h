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

#ifndef SNELL_RAY_TRACING_H
#define SNELL_RAY_TRACING_H

#include <Eigen/Dense>


// here, we assume that the origin is at (0, 0), and the points all have x > 0
std::pair<Eigen::VectorXd, Eigen::MatrixXd> trace_multiple_layers(const Eigen::VectorXd& layer_depths, const Eigen::VectorXd& layer_speeds, const Eigen::MatrixXd& end_points);
std::pair<double, Eigen::VectorXd> trace_single_layers(const Eigen::VectorXd& layer_depths, const Eigen::VectorXd& layer_speeds, const Eigen::Vector2d& end_point);
void visualize_rays(const Eigen::MatrixXd& end_points, const Eigen::VectorXd& layer_depths, Eigen::MatrixXd& layer_widths, double max_depth, bool wait=false, bool left=false);

#endif // SNELL_RAY_TRACING_H
