#include <gpgs_slam/cost_function.h>
#include <gpgs_slam/transforms.h>
#include <data_tools/submaps.h>

using namespace std;

void GaussianProcessCostFunction::get_transform_jacobian(Eigen::MatrixXd& J, const Eigen::Vector3d& x) const
{
    // Ok, let's write this out for the future
	// 3 is the X axis rotation, 4 Y axis, and 5 Z axis rotation
	// so rotating around Z(5), would give us higher Y, proportional to x(1)
	// and lower X, proportional to x(0)
	
    J.setZero();
    J.block<3, 3>(0, 0).setIdentity();
	
	// X axis rotations
    J(1, 3) = 0.*-x(2);
    J(2, 3) = 0.*x(1);
	// Y axis rotations
    J(2, 4) = 0.*-x(0);
    J(0, 4) = 0.*x(2);
	// Z axis rotations
    J(0, 5) = -x(1);
    J(1, 5) = x(0);

    J(2, 2) = 0.;
	//J.rightCols<3>() = 0.00000001*J.rightCols<3>();
	J.rightCols<3>() = 0.0000000000001*J.rightCols<3>();
	//J.rightCols<3>() = 0.0*J.rightCols<3>();
	
}

bool GaussianProcessCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Vector3d t1(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Matrix3d R1 = euler_to_matrix(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d t2(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Matrix3d R2 = euler_to_matrix(parameters[3][0], parameters[3][1], parameters[3][2]);

    cout << "Translation: " << t1.transpose() << ", Rotation: \n" << R1 << endl;

    Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465);
    Eigen::VectorXd ll;
    gp1.compute_neg_log_likelihoods(ll, points2in1.leftCols(2), points2in1.col(2));
    residuals[0] = -ll.mean();
    cout << "Residual: " << residuals[0] << endl;
    
    // Compute the Jacobian if asked for.
    if (jacobians != NULL) { // && jacobians[0] != NULL) {
        Eigen::MatrixXd dX;
        cout << "Computing derivatives..." << endl;
        //gp.compute_derivatives(dX, points.leftCols(2), points.col(2));
        gp1.compute_neg_log_derivatives(dX, points2in1.leftCols(2), points2in1.col(2));
        cout << "Done computing derivatives..." << endl;
        dX *= R1.transpose();

        Eigen::MatrixXd J(3, 6);
        Eigen::MatrixXd deltas(points2in1.rows(), 6);
        for (int m = 0; m < points2in1.rows(); ++m) {
            get_transform_jacobian(J, R1*points2in1.row(m).transpose()+t1);
            deltas.row(m) = dX.row(m)*J;
        }
        Eigen::RowVectorXd delta = deltas.colwise().mean();
        if (jacobians[0] != NULL) { 
            for (int j = 0; j < 3; ++j) {
                jacobians[0][j] = -delta[j];
            }
        }
        if (jacobians[1] != NULL) { 
            for (int j = 0; j < 3; ++j) {
                jacobians[1][j] = -delta[3+j];
            }
        }
        if (jacobians[2] != NULL) { 
            for (int j = 0; j < 3; ++j) {
                jacobians[2][j] = delta[j];
            }
        }
        if (jacobians[3] != NULL) { 
            for (int j = 0; j < 3; ++j) {
                jacobians[3][j] = delta[3+j];
            }
        }

        cout << "Derivative: " << delta << endl;
    }
    else {
        cout << "Not computing derivatives" << endl;
    }
    return true;
}
