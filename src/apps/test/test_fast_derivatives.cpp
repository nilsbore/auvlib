#include <Eigen/Dense>
#include <cxxopts.hpp>

// we should really do a types library, would avoid recompiling this all the time
#include <sparse_gp/gaussian_noise.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/sparse_gp.h>

#include <data_tools/data_structures.h>
#include <data_tools/submaps.h>
#include <gpgs_slam/gp_submaps.h>
#include <data_tools/transforms.h>

#include <gpgs_slam/cost_function.h>
#include <gpgs_slam/multi_visualizer.h>
#include <gpgs_slam/visualization.h>

#include <ceres/ceres.h>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <eigen_cereal/eigen_cereal.h>

#include <chrono>
#include <random>

using namespace std;
using namespace data_structures;

using TransT = vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using AngsT = vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using SubmapsGPT = gp_submaps::SubmapsGPT; // Process does not require aligned allocation as all matrices are dynamic
using ObsT = vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
using MatchesT = vector<pair<int, int>>; // tells us which maps overlap

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

void test_fast_derivatives(ObsT& points, SubmapsGPT& gps, TransT& trans, AngsT& rots, MatchesT& matches)
{
    using DerT = ObsT;
    using LLT = vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>;

    DerT slow_derivatives;
    LLT slow_likelihoods;

    // Get starting timepoint
    auto start = chrono::high_resolution_clock::now();
    for (const pair<int, int>& match : matches) {
        int i, j;
        tie(i, j) = match;
        cout << "Adding constraint between " << i << " and " << j << endl;

        Eigen::Vector3d R1 = rots[i];
        Eigen::Vector3d R2 = rots[j];
        Eigen::Vector3d t1 = trans[i];
        Eigen::Vector3d t2 = trans[j];

        Eigen::Matrix3d RM1 = data_transforms::euler_to_matrix(R1(0), R1(1), R1(2));
        Eigen::Matrix3d RM2 = data_transforms::euler_to_matrix(R2(0), R2(1), R2(2));

        Eigen::MatrixXd points2in1 = submaps::get_points_in_bound_transform(points[j], t2, RM2, t1, RM1, 465);
        Eigen::MatrixXd points1in2 = submaps::get_points_in_bound_transform(points[i], t1, RM1, t2, RM2, 465);

        Eigen::VectorXd ll1;
        gps[i].compute_neg_log_likelihoods(ll1, points2in1.leftCols<2>(), points2in1.col(2));
        Eigen::VectorXd ll2;
        gps[j].compute_neg_log_likelihoods(ll2, points1in2.leftCols<2>(), points1in2.col(2));

        slow_likelihoods.push_back(ll1);
        slow_likelihoods.push_back(ll2);

        Eigen::MatrixXd dX1;
        gps[i].compute_neg_log_derivatives(dX1, points2in1.leftCols<2>(), points2in1.col(2));
        Eigen::MatrixXd dX2;
        gps[j].compute_neg_log_derivatives(dX2, points1in2.leftCols<2>(), points1in2.col(2));

        slow_derivatives.push_back(dX1);
        slow_derivatives.push_back(dX2);
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
        tie(i, j) = match;
        cout << "Adding constraint between " << i << " and " << j << endl;

        Eigen::Vector3d R1 = rots[i];
        Eigen::Vector3d R2 = rots[j];
        Eigen::Vector3d t1 = trans[i];
        Eigen::Vector3d t2 = trans[j];

        Eigen::Matrix3d RM1 = data_transforms::euler_to_matrix(R1(0), R1(1), R1(2));
        Eigen::Matrix3d RM2 = data_transforms::euler_to_matrix(R2(0), R2(1), R2(2));

        Eigen::MatrixXd points2in1 = submaps::get_points_in_bound_transform(points[j], t2, RM2, t1, RM1, 465);
        Eigen::MatrixXd points1in2 = submaps::get_points_in_bound_transform(points[i], t1, RM1, t2, RM2, 465);

        Eigen::VectorXd ll1;
        Eigen::VectorXd ll2;
        Eigen::MatrixXd dX1;
        Eigen::MatrixXd dX2;
        gps[i].compute_neg_log_derivatives_fast(ll1, dX1, points2in1.leftCols<2>(), points2in1.col(2), true);
        gps[j].compute_neg_log_derivatives_fast(ll2, dX2, points1in2.leftCols<2>(), points1in2.col(2), true);

        fast_likelihoods.push_back(ll1);
        fast_likelihoods.push_back(ll2);
        fast_derivatives.push_back(dX1);
        fast_derivatives.push_back(dX2);
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
        cout << "Derivative i absolute col 2 norm: " << slow_derivatives[i].col(2).norm() << endl;
        cout << "Likelihood i absolute norm: " << slow_likelihoods[i].norm() << endl;
        cout << "Derivative i difference col 0 norm: " << (slow_derivatives[i].col(0) - fast_derivatives[i].col(0)).norm() << endl;
        cout << "Derivative i difference col 1 norm: " << (slow_derivatives[i].col(1) - fast_derivatives[i].col(1)).norm() << endl;
        cout << "Derivative i difference col 2 norm: " << (slow_derivatives[i].col(2) - fast_derivatives[i].col(2)).norm() << endl;
        cout << "Likelihood i likelihood norm: " << (slow_likelihoods[i] - fast_likelihoods[i]).norm() << endl;
    }
}

// Example: ./visualize_process --folder ../scripts --lsq 100.0 --sigma 0.1 --s0 1.
int main(int argc, char** argv)
{
    string file_str;

    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    //options.positional_help("[optional args]").show_positional_help();
    options.add_options()("help", "Print help")("file", "Input file", cxxopts::value(file_str));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }
    if (result.count("file") == 0) {
        cout << "Please provide folder arg..." << endl;
        exit(0);
    }

    boost::filesystem::path path(file_str);
    cout << "Input file : " << path << endl;

    gp_submaps ss = read_data<gp_submaps>(path);

    for (Eigen::MatrixXd& p : ss.points) {
        subsample_cloud(p);
    }

    ss.matches = submaps::compute_matches(ss.trans, ss.rots, ss.bounds);

    test_fast_derivatives(ss.points, ss.gps, ss.trans, ss.angles, ss.matches);

    return 0;
}
