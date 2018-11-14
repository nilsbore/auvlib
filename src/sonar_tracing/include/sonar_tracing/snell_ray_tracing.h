#ifndef SNELL_RAY_TRACING_H
#define SNELL_RAY_TRACING_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

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

// here, we assume that the origin is at (0, 0), and the points all have x > 0
std::pair<Eigen::VectorXd, Eigen::MatrixXd> trace_multiple_layers(const Eigen::VectorXd& layer_depths, const Eigen::VectorXd& layer_speeds, const Eigen::MatrixXd& end_points);
std::pair<double, Eigen::VectorXd> trace_single_layers(const Eigen::VectorXd& layer_depths, const Eigen::VectorXd& layer_speeds, const Eigen::Vector2d& end_point);

#endif // SNELL_RAY_TRACING_H
