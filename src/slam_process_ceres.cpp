#include <Eigen/Dense>
#include <cxxopts.hpp>

// we should really do a types library, would avoid recompiling this all the time
#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/submaps.h>
#include <data_tools/data_structures.h>

#include <gpgs_slam/cost_function.h>
#include <gpgs_slam/visualization.h>
#include <gpgs_slam/multi_visualizer.h>
#include <gpgs_slam/igl_visualizer.h>

#include <ceres/ceres.h>

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <random>
#include <thread>
#include <future>

using namespace std;

void subsample_cloud(Eigen::MatrixXd& points, int subsample)
{
    int counter = 0;
    for (int i = 0; i < points.rows(); ++i) {
        if (i % subsample == 0) {
            points.row(counter) = points.row(i);
            ++counter;
        }
    }
    points.conservativeResize(counter, 3);
}

gp_submaps train_or_load_gps(double lsq, double sigma, double s0, const boost::filesystem::path& folder)
{
    boost::filesystem::path path("gp_submaps.cereal");

    if (boost::filesystem::exists(path)) {
		return read_data<gp_submaps>(path);
    }

    SubmapsT submaps = read_submaps(folder);
    for (int i = 0; i < submaps.size(); ++i) {
        submaps[i].resize(4);
    }
    submaps.resize(4);

    gp_submaps ss;
    for (const auto& row : submaps) {
        ss.points.insert(ss.points.end(), row.begin(), row.end());
    }
    for (int i = 0; i < ss.points.size(); ++i) {
        Eigen::Matrix2d bb;
        bb.row(0) << -465., -465.; // bottom left corner
        bb.row(1) << 465., 465.; // top right corner
        ss.bounds.push_back(bb);
    }

    for (int j = 1; j < submaps[0].size(); ++j) {
        int i = 0;
        ss.matches.push_back(make_pair(i*submaps[i].size()+j, i*submaps[i].size()+j-1));
    }
    for (int i = 1; i < submaps.size(); ++i) {
        int j = 0;
        ss.matches.push_back(make_pair(i*submaps[i].size()+j, (i-1)*submaps[i].size()+j));
    }
    for (int i = 1; i < submaps.size(); ++i) {
        for (int j = 1; j < submaps[i].size(); ++j) {
            ss.matches.push_back(make_pair(i*submaps[i].size()+j, i*submaps[i].size()+j-1));
            ss.matches.push_back(make_pair(i*submaps[i].size()+j, (i-1)*submaps[i].size()+j));
        }
    }
    
    // check if already available
    ss.trans.resize(ss.points.size());
	ss.rots.resize(ss.points.size());
	SubmapsGPT gps;
    for (int i = 0; i < ss.points.size(); ++i) {
        ProcessT gp(100, s0);
        gp.kernel.sigmaf_sq = sigma;
        gp.kernel.l_sq = lsq*lsq;
        gp.kernel.p(0) = gp.kernel.sigmaf_sq;
        gp.kernel.p(1) = gp.kernel.l_sq;
        // this will also centralize the points
        tie(ss.trans[i], ss.rots[i]) = train_gp(ss.points[i], gp);
        ss.gps.push_back(gp);
        cout << "Pushed back..." << endl;
    }
    
    for (const Eigen::Matrix3d& R : ss.rots) {
        Eigen::Vector3d a; a << 0., 0., 0.;
        ss.angles.push_back(a);
    }
    
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0., 1.);
    for (int i = 0; i < ss.points.size(); ++i) {
        ss.trans[i](0) += 30.*distribution(generator);
        ss.trans[i](1) += 30.*distribution(generator);
        ss.angles[i](2) += 0.2*distribution(generator);
        ss.rots[i] = Eigen::AngleAxisd(ss.angles[i](2), Eigen::Vector3d::UnitZ()).matrix();
    }

    write_data(ss, path);
    
    return ss;
}

void register_processes_ceres(gp_submaps& ss)
{
    ceres::Problem problem;

    for (const pair<int, int>& match : ss.matches) {
        int i, j;
        tie (i, j) = match;
        cout << "Adding constraint between " << i << " and " << j << endl;
        ceres::CostFunction* cost_function1 = new GaussianProcessCostFunction(ss.gps[i], ss.bounds[i], ss.points[j]);
        ceres::CostFunction* cost_function2 = new GaussianProcessCostFunction(ss.gps[j], ss.bounds[j], ss.points[i]);

        //ceres::LossFunction* loss_function = new ceres::SoftLOneLoss(5.);
        ceres::LossFunction* loss_function1 = new ceres::HuberLoss(.5);
        ceres::LossFunction* loss_function2 = new ceres::HuberLoss(5.);
        ceres::LossFunction* loss_function = NULL;
        problem.AddResidualBlock(cost_function1, loss_function, ss.trans[i].data(), ss.angles[i].data(),
                                                                ss.trans[j].data(), ss.angles[j].data());
        problem.AddResidualBlock(cost_function2, loss_function, ss.trans[j].data(), ss.angles[j].data(),
                                                                ss.trans[i].data(), ss.angles[i].data());
    }
    
    for (int i = 0; i < ss.trans.size(); ++i) {
        problem.SetParameterLowerBound(ss.trans[i].data(), 0, ss.trans[i](0) - 100.);
        problem.SetParameterLowerBound(ss.trans[i].data(), 1, ss.trans[i](1) - 100.);
        problem.SetParameterUpperBound(ss.trans[i].data(), 0, ss.trans[i](0) + 100.);
        problem.SetParameterUpperBound(ss.trans[i].data(), 1, ss.trans[i](1) + 100.);

        problem.SetParameterLowerBound(ss.angles[i].data(), 2, ss.angles[i](2) - M_PI);
        problem.SetParameterUpperBound(ss.angles[i].data(), 2, ss.angles[i](2) + M_PI);

        //problem.SetParameterBlockConstant(rots[i].data());
        ceres::SubsetParameterization *subset_parameterization = new ceres::SubsetParameterization(3, {0, 1});
        problem.SetParameterization(ss.angles[i].data(), subset_parameterization);
    }
    
    //problem.SetParameterBlockConstant(trans[4].data());
    //problem.SetParameterBlockConstant(rots[4].data());

    ceres::Solver::Options options;
    //options.callbacks.push_back(new MultiVisCallback(points, gps, trans, rots));
	IglVisCallback* vis = new IglVisCallback(ss.points, ss.gps, ss.trans, ss.angles, ss.bounds);
    vis->set_matches(ss.matches);
    //vis->display(); // display initial conditions before starting optimization

    options.callbacks.push_back(vis);
    options.max_num_iterations = 200;
    options.update_state_every_iteration = true;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    //options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    
	//vis->display();
	auto handle = std::async(std::launch::async, [&options, &problem, &summary]() {
		ceres::Solve(options, &problem, &summary);
	});
	vis->display();
	handle.get();
    delete vis; // it seems like memory of this is not handled by ceres

    std::cout << summary.FullReport() << '\n';

    std::cout << "Is usable?: " << summary.IsSolutionUsable() << std::endl;
}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string folder_str;
	double lsq = 100.;
	double sigma = 10.;
	double s0 = 1.;
    int subsample = 1;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("folder", "Folder", cxxopts::value(folder_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("subsample", "Subsampling rate", cxxopts::value(subsample))
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

    gp_submaps ss = train_or_load_gps(lsq, sigma, s0, folder);
	
    ObsT original_points = ss.points;
    for (Eigen::MatrixXd& p : ss.points) {
        subsample_cloud(p, subsample);
    }
    
    ss.matches = compute_matches(ss.trans, ss.rots, ss.bounds);

    register_processes_ceres(ss);

    ss.points = original_points;
    write_data(ss, boost::filesystem::path("gp_results.cereal"));

    return 0;
}
