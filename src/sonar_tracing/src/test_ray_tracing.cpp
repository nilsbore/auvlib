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
