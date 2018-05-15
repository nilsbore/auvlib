#include <Eigen/Dense>
#include <cxxopts.hpp>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/colormap.h>
#include <data_tools/submaps.h>

#include <gpgs_slam/cost_function.h>
#include <gpgs_slam/visualization.h>
#include <gpgs_slam/transforms.h>

using namespace std;

void subsample_cloud(Eigen::MatrixXd& points)
{
    int subsample = 37;
    int counter = 0;
    for (int i = 0; i < points.rows(); ++i) {
        if (i % subsample == 0) {
            points.row(counter) = points.row(i);
            ++counter;
        }
    }
    points.conservativeResize(counter, 3);
}

void get_transform_jacobian(MatrixXd& J, const Vector3d& x)
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
	//J.rightCols<3>() = 0.00001*J.rightCols<3>();
	J.rightCols<3>() = 0.0*J.rightCols<3>();
	
}

Eigen::VectorXd compute_step(Eigen::MatrixXd& points, ProcessT& gp,
		                     Eigen::Vector3d& t, Eigen::Matrix3d& R)
{
    MatrixXd dX;
	cout << "Computing derivatives..." << endl;
    //gp.compute_derivatives(dX, points.leftCols(2), points.col(2));
    gp.compute_neg_log_derivatives(dX, points.leftCols(2), points.col(2));
	//dX.col(2).setZero();
	//R = rotations[i].toRotationMatrix();
    //t = means[i];
	cout << "Done computing derivatives..." << endl;
	Eigen::VectorXd l;
	gp.compute_likelihoods(l, points.leftCols(2), points.col(2));
	cout << "Mean likelihood: " << l.mean() << endl;
	cout << "Mean derivative: " << dX.colwise().sum() << endl;
	dX *= R.transpose();
	//double added_derivatives = 0;
	Eigen::RowVectorXd delta(6);
	//delta.setZero();
	Eigen::MatrixXd J(3, 6);
	// This would be really easy to do as one operation
	cout << "Getting transform jacobians and accumulating..." << endl;
    cout << "J size: " << J.rows() << "x" << J.cols() << endl;
    cout << "Delta size: " << delta.rows() << "x" << delta.cols() << endl;
    cout << "dX row size: " << dX.row(0).rows() << "x" << dX.row(0).cols() << endl;
    Eigen::MatrixXd deltas(points.rows(), 6);
	for (int m = 0; m < points.rows(); ++m) {
        //points.col(m) = R*points.col(m) + t;
        get_transform_jacobian(J, R*points.row(m).transpose() + t);
        //delta = (added_derivatives/(added_derivatives+1.))*delta + 1./(added_derivatives+1.)*dX.row(m)*J;
        //++added_derivatives;
        deltas.row(m) = dX.row(m)*J;
	}
    delta = deltas.colwise().mean();
	cout << "Done accumulating..." << endl;
	return delta.transpose();
}

tuple<Vector3d, Matrix3d> update_step(Eigen::VectorXd& delta)
{
    double step = 1e-0;
    Eigen::Vector3d dt;
    Eigen::Matrix3d dR;
    Matrix3d Rx = Eigen::AngleAxisd(step*delta(3), Vector3d::UnitX()).matrix();
    Matrix3d Ry = Eigen::AngleAxisd(step*delta(4), Vector3d::UnitY()).matrix();
    Matrix3d Rz = Eigen::AngleAxisd(step*delta(5), Vector3d::UnitZ()).matrix();
    dR = Rx*Ry*Rz;
    dt = step*delta.head<3>().transpose();

	return make_tuple(dt, dR);
}

void register_processes(Eigen::MatrixXd& points1, ProcessT& gp1, Eigen::Vector3d& t1, Eigen::Vector3d& R1,
				        Eigen::MatrixXd& points2, ProcessT& gp2, Eigen::Vector3d& t2, Eigen::Vector3d& R2)
{
    VisCallback vis(points1, points2, gp1, gp2, t1, R1, t2, R2);
    Matrix3d RM1 = euler_to_matrix(R1(0), R1(1), R1(2));
    Matrix3d RM2 = euler_to_matrix(R2(0), R2(1), R2(2));

    Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points2, t2, RM2, t1, RM1, 465);
    VectorXd delta(6);
	delta.setZero();
    bool delta_diff_small = false;
    while (true) { //!delta_diff_small) {
		Eigen::VectorXd delta_old = delta;
		cout << "Computing registration delta" << endl;
		delta = compute_step(points2in1, gp1, t1, RM1);
        delta_diff_small = (delta - delta_old).norm() < 1e-5f;
		Eigen::Vector3d dt;
		Eigen::Matrix3d dR;
		cout << "Computing registration update" << endl;
		tie(dt, dR) = update_step(delta);
		cout << "Registration update: " << delta << endl;
        RM1 = dR*RM1; // add to total rotation
        t1 += dt; // add to total translation
        points2in1 = get_points_in_bound_transform(points2, t2, RM2, t1, RM1, 465);
        vis.visualizer_step(RM1);
    }
}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string folder_str;
	double lsq = 100.;
	double sigma = 10.;
	double s0 = 1.;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	//options.positional_help("[optional args]").show_positional_help();
	options.add_options()
      ("help", "Print help")
      ("folder", "Folder", cxxopts::value(folder_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("s0", "Measurement noise", cxxopts::value(s0));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("folder") == 0) {
		cout << "Please provide folder arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path folder(folder_str);
	cout << "Folder : " << folder << endl;

	Eigen::MatrixXd points1 = read_submap(folder / "patch_00_00.xyz");
	Eigen::MatrixXd points2 = read_submap(folder / "patch_00_01.xyz");

	Eigen::Vector3d t1, t2;
	Eigen::Matrix3d RM1, RM2;
	
	ProcessT gp1(100, s0);
	gp1.kernel.sigmaf_sq = sigma;
	gp1.kernel.l_sq = lsq*lsq;
    gp1.kernel.p(0) = gp1.kernel.sigmaf_sq;
    gp1.kernel.p(1) = gp1.kernel.l_sq;
	tie(t1, RM1) = train_gp(points1, gp1);
    //R1 = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).matrix();
	t1.array() += -70.0;
    t1(2) -= -70.0; 
	
	ProcessT gp2(100, s0);
	gp2.kernel.sigmaf_sq = sigma;
	gp2.kernel.l_sq = lsq*lsq;
    gp2.kernel.p(0) = gp2.kernel.sigmaf_sq;
    gp2.kernel.p(1) = gp2.kernel.l_sq;
	tie(t2, RM2) = train_gp(points2, gp2);

	Eigen::Vector3d R1; R1 << 0., 0., 0.;
    Eigen::Vector3d R2; R2 << 0., 0., 0.;
    
    subsample_cloud(points1);
    subsample_cloud(points2);
	register_processes(points1, gp1, t1, R1, points2, gp2, t2, R2);

    return 0;
}
