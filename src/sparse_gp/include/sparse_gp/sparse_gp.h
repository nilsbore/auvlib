#ifndef SPARSE_GP_H
#define SPARSE_GP_H

#include <eigen3/Eigen/Dense>
#include <vector>

template <class Kernel, class Noise>
class sparse_gp
{
public:
    typedef Kernel kernel_type;
    typedef Noise noise_type;
public:
    kernel_type kernel; // the gp kernel type used
    noise_type noise; // the noise model used
private:
    // parameters of covariance function:
    int total_count; // How many points have I seen?
    int current_size; // how many inducing points do I have
    int capacity; // maximum number of inducing points
    double s20; // measurement noise, should be in noise instead
    double eps_tol; // error tolerance
    Eigen::VectorXd alpha; // Alpha and C are the parameters of the GP
    // Alpha is NxDout
    Eigen::MatrixXd C;
    Eigen::MatrixXd Q; // Inverse Gram Matrix (K_t).  C and Q are NxN
    Eigen::MatrixXd BV; // The Basis Vectors, BV is 2xN
    void add(const Eigen::VectorXd& X, double y);
    void delete_bv(int loc);
    double predict(const Eigen::VectorXd& X_star, double& sigma, bool conf);
    void construct_covariance(Eigen::VectorXd& K, const Eigen::Vector2d& X, const Eigen::MatrixXd& Xv);
    void shuffle(std::vector<int>& ind, int n);
    //void likelihood_dx(Eigen::Vector3d& dx, const Eigen::VectorXd& x, double y);
    void neg_log_likelihood_dx(Eigen::Vector3d& dx, const Eigen::VectorXd& x, double y);
    void likelihood_dtheta(Eigen::VectorXd& dtheta, const Eigen::Vector2d& x, double y);
    //double likelihood(const Eigen::Vector2d& x, double y);
    //double neg_log_likelihood(const Eigen::Vector2d& x, double y);
public:
    void reset(int new_capacity=100);
    void train_parameters(const Eigen::MatrixXd& X, const Eigen::VectorXd& y);
    void train_log_parameters(const Eigen::MatrixXd& X, const Eigen::VectorXd& y);
    //void compute_derivatives_fast(Eigen::MatrixXd& dX, const Eigen::MatrixXd& X, const Eigen::VectorXd& y);
    int size();
    void add_measurements(const Eigen::MatrixXd& X,const Eigen::VectorXd& y);
    void predict_measurements(Eigen::VectorXd& f_star, const Eigen::MatrixXd& X_star,
                              Eigen::VectorXd& sigconf, bool conf = false);
    double neg_log_likelihood(const Eigen::VectorXd& X_star, double y);
    //void compute_derivatives(Eigen::MatrixXd& dX, const Eigen::MatrixXd& X, const Eigen::VectorXd& y);
    void compute_neg_log_derivatives(Eigen::MatrixXd& dX, const Eigen::MatrixXd& X, const Eigen::VectorXd& y);
    void compute_neg_log_theta_derivatives_fast(Eigen::VectorXd& ll, Eigen::VectorXd& dtheta, const Eigen::MatrixXd& X, const Eigen::VectorXd& y);
    void compute_neg_log_derivatives_fast(Eigen::VectorXd& ll, Eigen::MatrixXd& dX, const Eigen::MatrixXd& X,
                                          const Eigen::VectorXd& y, bool compute_derivatives=true);
    //void compute_likelihoods(Eigen::VectorXd& l, const Eigen::MatrixXd& X, const Eigen::VectorXd& y);
    void compute_neg_log_likelihoods(Eigen::VectorXd& l, const Eigen::MatrixXd& X, const Eigen::VectorXd& y);
    //sparse_gp(int capacity = 30, double s0 = 1e-1f, double sigmaf = 1e-2f, double l = 0.08*0.08);
    sparse_gp(int capacity = 100, double s0 = 1e-1f);
    //sparse_gp(const sparse_gp& other);
	
    template <class Archive>
    void serialize(Archive & ar)
    {
       ar(kernel, noise, total_count, current_size,
          capacity, s20, eps_tol, alpha, C, Q, BV);
    }
};

//#include "impl/sparse_gp.hpp"

#endif // SPARSE_GP_H
