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
using TransT = vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsT = vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
using AngsT = vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using SubmapsGPT = vector<ProcessT>; // Process does not require aligned allocation as all matrices are dynamic
using ObsT = vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;
//using MatchesT = vector<pair<int, int> >; // tells us which maps overlap

void subsample_cloud(Eigen::MatrixXd& points)
{
    int subsample = 1; //37; // Works best
    int counter = 0;
    for (int i = 0; i < points.rows(); ++i) {
        if (i % subsample == 0) {
            points.row(counter) = points.row(i);
            ++counter;
        }
    }
    points.conservativeResize(counter, 3);
}

tuple<ObsT, SubmapsGPT, TransT, RotsT, MatchesT, BBsT> train_or_load_gps(double lsq, double sigma, double s0,
                                                                         const boost::filesystem::path& folder)
{
    if (boost::filesystem::exists("gp_submaps.cereal")) {
		TransT trans;
		RotsT rots;
		SubmapsGPT gps;
        ObsT obs;
        MatchesT matches;
        BBsT bounds;
        std::ifstream is("gp_submaps.cereal", std::ifstream::binary);
        {
			cereal::BinaryInputArchive archive(is);
			archive(obs, gps, trans, rots, matches, bounds);
        }
        is.close();
		return make_tuple(obs, gps, trans, rots, matches, bounds);
    }
    

    SubmapsT submaps = read_submaps(folder);
    for (int i = 0; i < submaps.size(); ++i) {
        submaps[i].resize(4);
    }
    submaps.resize(4);

    ObsT obs;
    for (const auto& row : submaps) {
        obs.insert(obs.end(), row.begin(), row.end());
    }
    BBsT bounds;
    for (int i = 0; i < obs.size(); ++i) {
        Eigen::Matrix2d bb;
        bb.row(0) << -465., -465.; // bottom left corner
        bb.row(1) << 465., 465.; // top right corner
        bounds.push_back(bb);
    }

    MatchesT matches;
    for (int j = 1; j < submaps[0].size(); ++j) {
        int i = 0;
        matches.push_back(make_pair(i*submaps[i].size()+j, i*submaps[i].size()+j-1));
    }
    for (int i = 1; i < submaps.size(); ++i) {
        int j = 0;
        matches.push_back(make_pair(i*submaps[i].size()+j, (i-1)*submaps[i].size()+j));
    }
    for (int i = 1; i < submaps.size(); ++i) {
        for (int j = 1; j < submaps[i].size(); ++j) {
            matches.push_back(make_pair(i*submaps[i].size()+j, i*submaps[i].size()+j-1));
            matches.push_back(make_pair(i*submaps[i].size()+j, (i-1)*submaps[i].size()+j));
        }
    }

    // check if already available
    TransT trans(obs.size());
	RotsT rots(obs.size());
	SubmapsGPT gps;
    for (int i = 0; i < obs.size(); ++i) {
        ProcessT gp(100, s0);
        gp.kernel.sigmaf_sq = sigma;
        gp.kernel.l_sq = lsq*lsq;
        gp.kernel.p(0) = gp.kernel.sigmaf_sq;
        gp.kernel.p(1) = gp.kernel.l_sq;
        // this will also centralize the points
        tie(trans[i], rots[i]) = train_gp(obs[i], gp);
        gps.push_back(gp);
        cout << "Pushed back..." << endl;
    }

	// write to disk for next time
    std::ofstream os("gp_submaps.cereal", std::ofstream::binary);
	{
		cereal::BinaryOutputArchive archive(os);
        archive(obs, gps, trans, rots, matches, bounds);
	}
    os.close();
    
    return make_tuple(obs, gps, trans, rots, matches, bounds);
}

void register_processes_ceres(ObsT& points, SubmapsGPT& gps, TransT& trans, AngsT& rots, MatchesT& matches, BBsT& bounds)
{
    ceres::Problem problem;

    for (const pair<int, int>& match : matches) {
        int i, j;
        tie (i, j) = match;
        cout << "Adding constraint between " << i << " and " << j << endl;
        ceres::CostFunction* cost_function1 = new GaussianProcessCostFunction(gps[i], bounds[i], points[j]);
        ceres::CostFunction* cost_function2 = new GaussianProcessCostFunction(gps[j], bounds[j], points[i]);

        //ceres::LossFunction* loss_function = new ceres::SoftLOneLoss(5.);
        ceres::LossFunction* loss_function1 = new ceres::HuberLoss(.5);
        ceres::LossFunction* loss_function2 = new ceres::HuberLoss(5.);
        ceres::LossFunction* loss_function = NULL;
        problem.AddResidualBlock(cost_function1, loss_function, trans[i].data(), rots[i].data(),
                                                                trans[j].data(), rots[j].data());
        problem.AddResidualBlock(cost_function2, loss_function, trans[j].data(), rots[j].data(),
                                                                trans[i].data(), rots[i].data());
    }
    
    for (int i = 0; i < trans.size(); ++i) {
        problem.SetParameterLowerBound(trans[i].data(), 0, trans[i](0) - 100.);
        problem.SetParameterLowerBound(trans[i].data(), 1, trans[i](1) - 100.);
        problem.SetParameterUpperBound(trans[i].data(), 0, trans[i](0) + 100.);
        problem.SetParameterUpperBound(trans[i].data(), 1, trans[i](1) + 100.);
        //problem.SetParameterBlockConstant(rots[i].data());
        ceres::SubsetParameterization *subset_parameterization = new ceres::SubsetParameterization(3, {0, 1});
        problem.SetParameterization(rots[i].data(), subset_parameterization);
    }
    
    //problem.SetParameterBlockConstant(trans[4].data());
    //problem.SetParameterBlockConstant(rots[4].data());

    ceres::Solver::Options options;
    //options.callbacks.push_back(new MultiVisCallback(points, gps, trans, rots));
	IglVisCallback* vis = new IglVisCallback(points, gps, trans, rots, bounds);
    options.callbacks.push_back(vis);
    options.max_num_iterations = 200;
    options.update_state_every_iteration = true;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    //options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    
	auto handle = std::async(std::launch::async, [&options, &problem, &summary]() {
		ceres::Solve(options, &problem, &summary);
	});
	vis->display();
	handle.get();

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
    ObsT points;
    SubmapsGPT gps;
    TransT trans;
    RotsT rots;
    MatchesT matches;
    BBsT bounds;
    tie(points, gps, trans, rots, matches, bounds) = train_or_load_gps(lsq, sigma, s0, folder);
	
    for (Eigen::MatrixXd& p : points) {
        subsample_cloud(p);
    }

    AngsT angles;
    for (const Eigen::Matrix3d& R : rots) {
        Eigen::Vector3d a; a << 0., 0., 0.;
        angles.push_back(a);
    }
    
    CloudT::Ptr cloud(new CloudT);
    for (int i = 0; i < points.size(); ++i) {
        CloudT::Ptr subcloud = construct_submap_and_gp_cloud(points[i], gps[i], trans[i], rots[i], 2*i);
        *cloud += *subcloud;
    }
	/*pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped ())
	{
	}*/
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0., 1.);
    for (int i = 0; i < points.size(); ++i) {
        trans[i](0) += 30.*distribution(generator);
        trans[i](1) += 30.*distribution(generator);
        angles[i](2) += 0.2*distribution(generator);
        rots[i] = Eigen::AngleAxisd(angles[i](2), Eigen::Vector3d::UnitZ()).matrix();
    }

    matches = compute_matches(trans, rots, bounds);
    register_processes_ceres(points, gps, trans, angles, matches, bounds);

    return 0;
}
