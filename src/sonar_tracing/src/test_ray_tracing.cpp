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

#include <sonar_tracing/snell_ray_tracing.h>
#include <cxxopts.hpp>

using namespace std;

int main(int argc, char** argv)
{
    Eigen::MatrixXd end_points(10, 2);
    end_points << 3., -40.1,
                  13., -40.1,
                  23., -40.1,
                  33., -40.1,
                  43., -32.1,
                  53., -22.1,
                  63., -15.1,
                  73., -22.1,
                  83., -30.1,
                  93., -40.1;
    Eigen::VectorXd layer_speeds(4); layer_speeds << 0.8, 0.9, 1.1, 1.3;
    Eigen::VectorXd layer_depths(3); layer_depths << -10., -20., -30.;

    // here, we assume that the origin is at (0, 0), and the points all have x > 0
    Eigen::VectorXd end_times;
    Eigen::MatrixXd layer_widths;
    tie(end_times, layer_widths) = trace_multiple_layers(layer_depths, layer_speeds, end_points);
    
    cout << "Got final times: " << end_times.transpose() << endl;

    cout << "Time of first ray: " << end_points.row(0).norm()/1. << endl;

    visualize_rays(end_points, layer_depths, layer_widths, -45.);

    return 0;
}
