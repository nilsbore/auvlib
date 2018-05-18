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
#include <gpgs_slam/transforms.h>

#include <ceres/ceres.h>

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

#include <random>
#include <chrono>

using namespace std;
using TransT = vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsT = vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;
using AngsT = vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using SubmapsGPT = vector<ProcessT>; // Process does not require aligned allocation as all matrices are dynamic
using ObsT = vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;
using MatchesT = vector<pair<int, int> >; // tells us which maps overlap

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

tuple<ObsT, SubmapsGPT, TransT, RotsT, MatchesT> train_or_load_gps(double lsq, double sigma, double s0,
                                                                   const boost::filesystem::path& folder)
{
    if (boost::filesystem::exists("gp_submaps.cereal")) {
		TransT trans;
		RotsT rots;
		SubmapsGPT gps;
        ObsT obs;
        MatchesT matches;
        std::ifstream is("gp_submaps.cereal", std::ifstream::binary);
        {
			cereal::BinaryInputArchive archive(is);
			archive(obs, gps, trans, rots, matches);
        }
        is.close();
		return make_tuple(obs, gps, trans, rots, matches);
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
        archive(obs, gps, trans, rots, matches);
	}
    os.close();
    
    return make_tuple(obs, gps, trans, rots, matches);
}

void test_fast_derivatives(ObsT& points, SubmapsGPT& gps, TransT& trans, AngsT& rots, MatchesT& matches)
{
    using DerT = ObsT;
    using LLT = vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> >;

    DerT slow_derivatives;
    LLT slow_likelihoods;

    // Get starting timepoint
    auto start = chrono::high_resolution_clock::now();
    for (const pair<int, int>& match : matches) {
        int i, j;
        tie (i, j) = match;
        cout << "Adding constraint between " << i << " and " << j << endl;

        Eigen::Vector3d R1 = rots[i];
        Eigen::Vector3d R2 = rots[j];
        Eigen::Vector3d t1 = trans[i];
        Eigen::Vector3d t2 = trans[j];

        Eigen::Matrix3d RM1 = euler_to_matrix(R1(0), R1(1), R1(2));
        Eigen::Matrix3d RM2 = euler_to_matrix(R2(0), R2(1), R2(2));

        Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points[j], t2, RM2, t1, RM1, 465);
        Eigen::MatrixXd points1in2 = get_points_in_bound_transform(points[i], t1, RM1, t2, RM2, 465);

        Eigen::VectorXd ll1;
        gps[i].compute_neg_log_likelihoods(ll1, points2in1.leftCols<2>(), points2in1.col(2));
        Eigen::VectorXd ll2;
        gps[j].compute_neg_log_likelihoods(ll2, points1in2.leftCols<2>(), points1in2.col(2));

        slow_likelihoods.push_back(ll1); slow_likelihoods.push_back(ll2);

        Eigen::MatrixXd dX1;
        gps[i].compute_neg_log_derivatives(dX1, points2in1.leftCols<2>(), points2in1.col(2));
        Eigen::MatrixXd dX2;
        gps[j].compute_neg_log_derivatives(dX2, points1in2.leftCols<2>(), points1in2.col(2));

        slow_derivatives.push_back(dX1); slow_derivatives.push_back(dX2);
    }
    // Get ending timepoint
    auto stop = chrono::high_resolution_clock::now();
 
    // Get duration. Substart timepoints to 
    // get durarion. To cast it to proper unit
    // use duration cast method
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
 
    cout << "Time taken by slow_likelihoods: " << duration.count() << " microseconds" << endl;
    
    DerT fast_derivatives;
    LLT fast_likelihoods;

    // Get starting timepoint
    start = chrono::high_resolution_clock::now();
    for (const pair<int, int>& match : matches) {
        int i, j;
        tie (i, j) = match;
        cout << "Adding constraint between " << i << " and " << j << endl;

        Eigen::Vector3d R1 = rots[i];
        Eigen::Vector3d R2 = rots[j];
        Eigen::Vector3d t1 = trans[i];
        Eigen::Vector3d t2 = trans[j];

        Eigen::Matrix3d RM1 = euler_to_matrix(R1(0), R1(1), R1(2));
        Eigen::Matrix3d RM2 = euler_to_matrix(R2(0), R2(1), R2(2));

        Eigen::MatrixXd points2in1 = get_points_in_bound_transform(points[j], t2, RM2, t1, RM1, 465);
        Eigen::MatrixXd points1in2 = get_points_in_bound_transform(points[i], t1, RM1, t2, RM2, 465);

        Eigen::VectorXd ll1;
        Eigen::VectorXd ll2;
        Eigen::MatrixXd dX1;
        Eigen::MatrixXd dX2;
        gps[i].compute_neg_log_derivatives_fast(ll1, dX1, points2in1.leftCols<2>(), points2in1.col(2), true);
        gps[j].compute_neg_log_derivatives_fast(ll2, dX2, points1in2.leftCols<2>(), points1in2.col(2), true);

        fast_likelihoods.push_back(ll1); fast_likelihoods.push_back(ll2);
        fast_derivatives.push_back(dX1); fast_derivatives.push_back(dX2);
    }
    // Get ending timepoint
    stop = chrono::high_resolution_clock::now();
 
    // Get duration. Substart timepoints to 
    // get durarion. To cast it to proper unit
    // use duration cast method
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
 
    cout << "Time taken by fast_likelihoods: " << duration.count() << " microseconds" << endl;

    for (int i = 0; i < slow_derivatives.size(); ++i) {
        cout << "Derivative i absolute col 0 norm: " << slow_derivatives[i].col(0).norm() << endl;
        cout << "Derivative i absolute col 1 norm: " << slow_derivatives[i].col(1).norm() << endl;
        cout << "Likelihood i absolute norm: " << slow_likelihoods[i].norm() << endl;
        cout << "Derivative i difference col 0 norm: " << (slow_derivatives[i].col(0) - fast_derivatives[i].col(0)).norm() << endl;
        cout << "Derivative i difference col 1 norm: " << (slow_derivatives[i].col(1) - fast_derivatives[i].col(1)).norm() << endl;
        cout << "Likelihood i likelihood norm: " << (slow_likelihoods[i] - fast_likelihoods[i]).norm() << endl;
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

    //SubmapsT submaps = read_submaps(folder);
	//visualize_submaps(submaps);
    ObsT points;
    SubmapsGPT gps;
    TransT trans;
    RotsT rots;
    MatchesT matches;
    tie(points, gps, trans, rots, matches) = train_or_load_gps(lsq, sigma, s0, folder);
	
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
    }

    test_fast_derivatives(points, gps, trans, angles, matches);

    return 0;
}
