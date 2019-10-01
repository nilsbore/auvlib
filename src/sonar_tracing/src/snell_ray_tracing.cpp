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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#ifndef _MSC_VER
#ifdef WITH_CERES
  #include <ceres/ceres.h>
#endif

using namespace std;

// here, we assume that the origin is at (0, 0), and the points all have x > 0
pair<Eigen::VectorXd, Eigen::MatrixXd> trace_multiple_layers(const Eigen::VectorXd& layer_depths, const Eigen::VectorXd& layer_speeds, const Eigen::MatrixXd& end_points)
{
    Eigen::MatrixXd layer_widths(layer_depths.rows()+2, end_points.rows());
    Eigen::VectorXd end_times(end_points.rows());
    for (int i = 0; i < end_points.rows(); ++i) {
        double ray_time;
        Eigen::VectorXd point_results;
        tie(ray_time, point_results) = trace_single_layers(layer_depths, layer_speeds, end_points.row(i).transpose());
        end_times(i) = ray_time;
        layer_widths.col(i).setConstant(point_results(point_results.rows()-1));
        layer_widths.col(i).head(point_results.rows()) = point_results;
    }
    return make_pair(end_times, layer_widths);
}

//#ifndef _MSC_VER
#ifdef WITH_CERES
class LayerWidthCostFunctor {
public:
    LayerWidthCostFunctor(double height, double speed) : height(height), speed(speed)
    {
    }

    template <typename T>
    bool operator()(const T* const x1, const T* const x2, T* e) const
    {
        e[0] = pow((*x2-*x1)*(*x2-*x1)+T(height*height), 0.25)/T(sqrt(speed));
        return true;
    }

    static ceres::CostFunction* Create(double height, double speed)
    {
        return new ceres::AutoDiffCostFunction<LayerWidthCostFunctor, 1, 1, 1>(
            new LayerWidthCostFunctor(height, speed));
    }

private:
    double height;
    double speed;
};
#endif

pair<double, Eigen::VectorXd> trace_single_layers(const Eigen::VectorXd& layer_depths, const Eigen::VectorXd& layer_speeds, const Eigen::Vector2d& end_point)
{
//#ifndef _MSC_VER
#ifdef WITH_CERES
    Eigen::VectorXd layer_widths(layer_depths.rows()+2);
    layer_widths(0) = 0.;

    int i;
    for (i = 0; i < layer_depths.rows(); ++i) {
        if (layer_depths(i) < end_point(1)) {
            break;
        }
        layer_widths(i+1) = (end_point(0) - 0.)/(end_point(1) - 0.)*(layer_depths(i) - 0.);
    }
    const int nbr_layers = i+2;
    layer_widths.conservativeResize(nbr_layers);
    layer_widths(nbr_layers-1) = end_point(0);

    Eigen::VectorXd layer_depths_full(nbr_layers);
    layer_depths_full(0) = 0.;
    layer_depths_full.segment(1, i) = layer_depths.head(i);
    layer_depths_full(nbr_layers-1) = end_point(1);

    ceres::Problem problem;
    vector<ceres::ResidualBlockId> residual_block_ids;

    for (int i = 1; i < nbr_layers; ++i) {
        cout << "Adding binary constraint between " << i-1 << " and " << i << endl;

        ceres::CostFunction* cost_function = LayerWidthCostFunctor::Create(layer_depths_full[i]-layer_depths_full[i-1], layer_speeds[i-1]);
        ceres::LossFunction* loss_function = NULL;
        residual_block_ids.push_back(problem.AddResidualBlock(cost_function, loss_function, &layer_widths[i-1], &layer_widths[i]));
        problem.SetParameterLowerBound(&layer_widths[i], 0, 0.);
        problem.SetParameterUpperBound(&layer_widths[i], 0, end_point[0]);
    }
    
    problem.SetParameterBlockConstant(&layer_widths[0]); // start point
    problem.SetParameterBlockConstant(&layer_widths[nbr_layers-1]); // end point

    ceres::Solver::Options options;
	//IglVisCallback* vis = new IglVisCallback(ss.points, ss.gps, ss.trans, ss.angles, ss.bounds);
    //options.callbacks.push_back(vis);

    options.max_num_iterations = 200;
    options.update_state_every_iteration = true; // this is only for visualization
    options.num_threads = 8;
    //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    //options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	//vis->display();
    //delete vis; // it seems like memory of this is not handled by ceres
    
    cout << summary.FullReport() << endl;
    cout << "Is usable?: " << summary.IsSolutionUsable() << endl;

    // return the final ray time and the optimized intermediary points
    return make_pair(2.*summary.final_cost, layer_widths);
#endif
    return make_pair(0., Eigen::VectorXd::Zero(layer_depths.rows()));
}

void visualize_rays(const Eigen::MatrixXd& end_points, const Eigen::VectorXd& layer_depths,
                    Eigen::MatrixXd& layer_widths, double max_depth, bool wait, bool left)
{
    const int image_width = 1000;
    const int image_height = 400;

    cv::Mat image(image_height, image_width, CV_8UC3, cv::Scalar(255, 255, 255));

    double vscale = double(image_height)/max_depth;
    double hscale = -vscale;

    for (int i = 0; i < end_points.rows(); ++i) {
        cv::Point center(hscale*end_points(i, 0), vscale*end_points(i, 1));
        cv::circle(image, center, 5, cv::Scalar(255, 0, 0), 1);
        cv::line(image, cv::Point(0, 0), center, cv::Scalar(255, 0, 0), 1);
    }

    for (int i = 0; i < layer_depths.rows(); ++i) {
        int row = int(vscale*layer_depths(i));
        cv::line(image, cv::Point(0, row), cv::Point(image_width-1, row), cv::Scalar(0, 0, 255), 1);
    }

    for (int i = 0; i < end_points.rows(); ++i) {
        cv::Point last_point(0, 0);
        for (int j = 0; j < layer_depths.rows(); ++j) {
            if (end_points(i, 1) > layer_depths(j)) {
                break;
            }
            cv::Point new_point(hscale*layer_widths(j+1, i), vscale*layer_depths(j));
            cv::line(image, last_point, new_point, cv::Scalar(0, 255, 0), 1);
            last_point = new_point;
        }
        cv::Point new_point(hscale*end_points(i, 0), vscale*end_points(i, 1));
        cv::line(image, last_point, new_point, cv::Scalar(0, 255, 0), 1);
    }

    if (left) {
        cv::flip(image, image, +1);
        cv::imshow("Left sidescan", image);
    }
    else {
        cv::imshow("Right sidescan", image);
    }
    if (wait) {
        cv::waitKey();
    }
    else {
        cv::waitKey(100);
    }
}
