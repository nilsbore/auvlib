#include <gpgs_slam/cost_function.h>
#include <data_tools/transforms.h>
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

pair<vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >,
     vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >
GaussianProcessCostFunction::get_new_transform_jacobian(const Eigen::MatrixXd& X,
                                                        const Eigen::Vector3d& t1, const Eigen::Vector3d& rot1,
                                                        const Eigen::Vector3d& t2, const Eigen::Vector3d& rot2) const
{
    // Ok, let's write this out for the future
	// 3 is the X axis rotation, 4 Y axis, and 5 Z axis rotation
	// so rotating around Z(5), would give us higher Y, proportional to x(1)
	// and lower X, proportional to x(0)
    vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > R1s = euler_to_matrices(rot1(0), rot1(1), rot1(2));
    vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > R2s = euler_to_matrices(rot2(0), rot2(1), rot2(2));
    vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > R1ds = euler_to_diff_matrices(rot1(0), rot1(1), rot1(2));
    vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > R2ds = euler_to_diff_matrices(rot2(0), rot2(1), rot2(2));
    for (int i = 0; i < 3; ++i) {
        R1s[i].transposeInPlace();
        R1ds[i].transposeInPlace();
    }
    Eigen::Matrix3d R1 = R1s[2]*R1s[1]*R1s[0];
    Eigen::Matrix3d R2 = R2s[0]*R2s[1]*R2s[2];
    
    vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > J1s; 
    vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > J2s; 
    for (int i = 0; i < X.rows(); ++i) {
        Eigen::Vector3d p = X.row(i).transpose();

        Eigen::MatrixXd J1(3, 6);
        Eigen::MatrixXd J2(3, 6);

        J2.leftCols<3>() = R1.transpose();
        J1.leftCols<3>() = -R1.transpose();

        J2.col(3) = R1*R2ds[0]*R2s[1]*R2s[2]*p;
        J2.col(4) = R1*R2s[0]*R2ds[1]*R2s[2]*p;
        J2.col(5) = R1*R2s[0]*R2s[1]*R2ds[2]*p;

        J1.col(3) = R1s[2]*R1s[1]*R1ds[0]*(R2*p+t2-t1);
        J1.col(4) = R1s[2]*R1ds[1]*R1s[0]*(R2*p+t2-t1);
        J1.col(5) = R1ds[2]*R1s[1]*R1s[0]*(R2*p+t2-t1);

        J1s.push_back(J1);
        J2s.push_back(J2);
    }
	
    return make_pair(J1s, J2s);
}

/*
Eigen::MatrixXd get_points_in_bound_transform(Eigen::MatrixXd points, Eigen::Vector3d& t,
				                              Eigen::Matrix3d& R, Eigen::Vector3d& t_in,
											  Eigen::Matrix3d& R_in, Eigen::Matrix2d& bounds)
{
    points *= R.transpose()*R_in;
	points.rowwise() += (t.transpose()*R_in - t_in.transpose()*R_in);

    int counter = 0;
	for (int i = 0; i < points.rows(); ++i) {
		if (points(i, 0) < bounds(1, 0) && points(i, 0) > bounds(0, 0) &&
		    points(i, 1) < bounds(1, 1) && points(i, 1) > bounds(0, 1)) {
		    points.row(counter) = points.row(i);
			++counter;
		}
    }
	points.conservativeResize(counter, 3);
	return points;
}
*/

bool GaussianProcessCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    Eigen::Vector3d t1(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d rot1(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Matrix3d R1 = euler_to_matrix(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d t2(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Vector3d rot2(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Matrix3d R2 = euler_to_matrix(parameters[3][0], parameters[3][1], parameters[3][2]);

    cout << "Translation: " << t1.transpose() << ", Rotation: \n" << R1 << endl;

    //Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, R2, t1, R1, bounds1);
    //Eigen::MatrixXd points2in1 = get_certain_points_in_bound_transform(points2, t2, R2, t1, R1, bounds1);
    Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465.);
    // NOTE: this is a workaround if there is no longer any overlap
    if (points2in1.rows() == 0) {
        points2in1 = points2*R2.transpose()*R1;
	    points2in1.rowwise() += (t2 - t1).transpose()*R1;
    }
    
    Eigen::MatrixXd dX;
    Eigen::VectorXd ll;

    bool compute_derivatives = jacobians != NULL;
    // TODO: we differentiate wrt the process, not the points, leading to a sign reversal. Fix this
    gp1.compute_neg_log_derivatives_fast(ll, dX, points2in1.leftCols<2>(), points2in1.col(2), compute_derivatives);

    //gp1.compute_neg_log_likelihoods(ll, points2in1.leftCols(2), points2in1.col(2));
    residuals[0] = 1.-ll.mean();
    //residuals[0] = -ll.mean();
    cout << "Residual: " << residuals[0] << endl;
    
    // Compute the Jacobian if asked for.
    if (compute_derivatives) { // && jacobians[0] != NULL) {
        cout << "Computing derivatives..." << endl;
        //gp.compute_derivatives(dX, points.leftCols(2), points.col(2));
        //gp1.compute_neg_log_derivatives(dX, points2in1.leftCols(2), points2in1.col(2));
        cout << "Done computing derivatives..." << endl;
        //dX *= R1.transpose();
    
        // transform back from coordinates system of 1 to 2
        Eigen::MatrixXd points2sub = points2in1*R1.transpose()*R2;
	    points2sub.rowwise() += (t1.transpose()*R2 - t2.transpose()*R2);
    
        vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > J1s; 
        vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > J2s;
        tie(J1s, J2s) = get_new_transform_jacobian(points2sub, t1, rot1, t2, rot2); 

        //Eigen::MatrixXd J(3, 6);
        Eigen::MatrixXd delta1s(points2sub.rows(), 6);
        Eigen::MatrixXd delta2s(points2sub.rows(), 6);
        for (int m = 0; m < points2sub.rows(); ++m) {
            //get_transform_jacobian(J, R1*points2in1.row(m).transpose()+t1);
            /*if (ll(m) < -1.3) {
                continue;
            }*/
            delta1s.row(m) = dX.row(m)*J1s[m];
            delta2s.row(m) = dX.row(m)*J2s[m];
        }
        Eigen::RowVectorXd delta1 = delta1s.colwise().mean();
        Eigen::RowVectorXd delta2 = delta2s.colwise().mean();
        if (jacobians[0] != NULL) {
            for (int j = 0; j < 3; ++j) {
                jacobians[0][j] = delta1[j];
            }
        }
        if (jacobians[1] != NULL) {
            for (int j = 0; j < 3; ++j) {
                //jacobians[1][j] = 0.0000000000001*delta1[3+j];
                jacobians[1][j] = delta1[3+j];
            }
        }
        if (jacobians[2] != NULL) {
            for (int j = 0; j < 3; ++j) {
                jacobians[2][j] = delta2[j];
            }
        }
        if (jacobians[3] != NULL) {
            for (int j = 0; j < 3; ++j) {
                //jacobians[3][j] = 0.0000000000001*delta2[3+j];
                jacobians[3][j] = delta2[3+j];
            }
        }

        cout << "Derivative: " << delta1 << ", " << delta2 << endl;
    }
    else {
        cout << "Not computing derivatives" << endl;
    }
    return true;
}
