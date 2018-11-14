#include <sonar_tracing/snell_ray_tracing.h>

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

pair<double, Eigen::VectorXd> trace_single_layers(const Eigen::VectorXd& layer_depths, const Eigen::VectorXd& layer_speeds, const Eigen::Vector2d& end_point)
{
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
    return make_pair(summary.final_cost, layer_widths);
}
