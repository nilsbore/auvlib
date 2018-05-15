#include <Eigen/Dense>
#include <cxxopts.hpp>

// we should really do a types library, would avoid recompiling this all the time
#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/submaps.h>

#include <gpgs_slam/cost_function.h>
#include <gpgs_slam/visualization.h>

#include <ceres/ceres.h>

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

void register_processes_ceres(Eigen::MatrixXd& points1, ProcessT& gp1, Eigen::Vector3d& t1, Eigen::Vector3d& R1,
                              Eigen::MatrixXd& points2, ProcessT& gp2, Eigen::Vector3d& t2, Eigen::Vector3d& R2)
{


    ceres::Problem problem;
    //ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);
    ceres::CostFunction* cost_function1 = new GaussianProcessCostFunction(gp1, points2);
    ceres::CostFunction* cost_function2 = new GaussianProcessCostFunction(gp2, points1);

    //ceres::LossFunction* loss_function = new ceres::SoftLOneLoss(1.);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.);
    ceres::LossFunction* loss_function = NULL;
    problem.AddResidualBlock(cost_function1, loss_function, t1.data(), R1.data(), t2.data(), R2.data());
    //problem.AddResidualBlock(cost_function2, loss_function, t2.data(), R2.data(), t1.data(), R1.data());

    /*problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
        quaternion_local_parameterization);
    problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
        quaternion_local_parameterization);*/
    
    problem.SetParameterLowerBound(t1.data(), 0, t1(0) - 100.);
    problem.SetParameterLowerBound(t1.data(), 1, t1(1) - 100.);
    problem.SetParameterUpperBound(t1.data(), 0, t1(0) + 100.);
    problem.SetParameterUpperBound(t1.data(), 1, t1(1) + 100.);
    
    problem.SetParameterLowerBound(t2.data(), 0, t2(0) - 100.);
    problem.SetParameterLowerBound(t2.data(), 1, t2(1) - 100.);
    problem.SetParameterUpperBound(t2.data(), 0, t2(0) + 100.);
    problem.SetParameterUpperBound(t2.data(), 1, t2(1) + 100.);
    
    //problem.SetParameterBlockConstant(t1.data());
    problem.SetParameterBlockConstant(t2.data());
    problem.SetParameterBlockConstant(R1.data());
    problem.SetParameterBlockConstant(R2.data());

    ceres::Solver::Options options;
    options.callbacks.push_back(new VisCallback(points1, points2, gp1, gp2, t1, R1, t2, R2));
    options.max_num_iterations = 200;
    options.update_state_every_iteration = true;
    //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << '\n';

    std::cout << "Is usable?: " << summary.IsSolutionUsable() << std::endl;

}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

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

    //SubmapsT submaps = read_submaps(folder);
	//visualize_submaps(submaps);
	
	Eigen::MatrixXd points1 = read_submap(folder / "patch_00_00.xyz");
	Eigen::MatrixXd points2 = read_submap(folder / "patch_00_01.xyz");
	//points = 0.1/930.*points;
	//visualize_submap(points);

	Eigen::Vector3d t1, t2; // translations
    Eigen::Vector3d R1, R2; // Euler angles
    R1 << 0., 0., 0.; //2;
    R2 << 0., 0., 0.;
	Eigen::Matrix3d RM1, RM2; // rotation matrices
	
	ProcessT gp1(100, s0);
	gp1.kernel.sigmaf_sq = sigma;
	gp1.kernel.l_sq = lsq*lsq;
    gp1.kernel.p(0) = gp1.kernel.sigmaf_sq;
    gp1.kernel.p(1) = gp1.kernel.l_sq;
	tie(t1, RM1) = train_gp(points1, gp1);
    //R1 = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).matrix();
    //RM1 = euler_to_matrix(R1(0), R1(1), R1(2));
    Eigen::Vector3d t1_gt = t1;
	t1.array() += -70.0;
    t1(2) -= -70.0; 
	
	ProcessT gp2(100, s0);
	gp2.kernel.sigmaf_sq = sigma;
	gp2.kernel.l_sq = lsq*lsq;
    gp2.kernel.p(0) = gp2.kernel.sigmaf_sq;
    gp2.kernel.p(1) = gp2.kernel.l_sq;
	tie(t2, RM2) = train_gp(points2, gp2);
    //RM2 = euler_to_matrix(R2(0), R2(1), R2(2));
    Eigen::Vector3d t2_gt = t2;

    subsample_cloud(points1);
    subsample_cloud(points2);
    //cv::Mat vis = visualize_likelihoods(gp1, t1, RM1, points2, t2, RM2);

	register_processes_ceres(points1, gp1, t1, R1, points2, gp2, t2, R2);

    cout << "T1 gt value: " << t1_gt.transpose() << endl;
    cout << "T1 final value: " << t1.transpose() << endl;
    cout << "T2 gt value: " << t2_gt.transpose() << endl;
    cout << "T2 final value: " << t2.transpose() << endl;

    return 0;
}
