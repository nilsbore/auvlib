#include <Eigen/Dense>
#include <cxxopts.hpp>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/std_data.h>
#include <data_tools/benchmark.h>
#include <data_tools/colormap.h>
#include <data_tools/submaps.h>
#include <data_tools/transforms.h>
#include <gpgs_slam/gp_submaps.h>

#include <gpgs_slam/cost_function.h>
#include <gpgs_slam/visualization.h>

using namespace std;
using namespace data_transforms;
using namespace std_data;

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

vector<pair<int, int> > get_medgaz_matches()
{
    vector<pair<int, int> > matches;
    for (int i = 0; i < 3; ++i) {

        for (int j = 3; j < 5; ++j) {
            matches.push_back(make_pair(i, j));
        }

        for (int j = 5; j < 7; ++j) {
            matches.push_back(make_pair(i, j));
        }
    }
    return matches;
}

pair<vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >,
     vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >
get_transform_jacobians(const Eigen::MatrixXd& X,
                        const Eigen::Vector3d& t1, const Eigen::Vector3d& rot1,
                        const Eigen::Vector3d& t2, const Eigen::Vector3d& rot2)
{
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

Eigen::VectorXd compute_step(Eigen::MatrixXd& points2, ProcessT& gp1,
                             Eigen::Vector3d& t1, Eigen::Vector3d& rot1,
                             Eigen::Vector3d& t2, Eigen::Vector3d& rot2)
{
    Eigen::Matrix3d R1 = euler_to_matrix(rot1[0], rot1[1], rot1[2]);
    Eigen::Matrix3d R2 = euler_to_matrix(rot2[0], rot2[1], rot2[2]);

    Eigen::MatrixXd points2in1 = submaps::get_points_in_bound_transform(points2, t2, R2, t1, R1, 465.);

    Eigen::VectorXd ll;
    Eigen::MatrixXd dX;
	cout << "Computing derivatives..." << endl;
    gp1.compute_neg_log_derivatives_fast(ll, dX, points2in1.leftCols(2), points2in1.col(2), true);
	cout << "Done computing derivatives..." << endl;
	cout << "Mean likelihood: " << ll.mean() << endl;
	cout << "Mean derivative: " << dX.colwise().sum() << endl;

    Eigen::MatrixXd points2sub = points2in1*R1.transpose()*R2;
    points2sub.rowwise() += (t1.transpose()*R2 - t2.transpose()*R2);

    vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > J1s; 
    vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > J2s;
    tie(J1s, J2s) = get_transform_jacobians(points2sub, t1, rot1, t2, rot2); 

    Eigen::MatrixXd delta1s(points2sub.rows(), 6);
    Eigen::MatrixXd delta2s(points2sub.rows(), 6);
    for (int m = 0; m < points2sub.rows(); ++m) {
        delta1s.row(m) = dX.row(m)*J1s[m];
        delta2s.row(m) = dX.row(m)*J2s[m];
    }
    Eigen::RowVectorXd delta1 = delta1s.colwise().mean();
    Eigen::RowVectorXd delta2 = delta2s.colwise().mean();

	return delta1.transpose();
}

void register_processes(Eigen::MatrixXd& points1, ProcessT& gp1, Eigen::Vector3d& t1, Eigen::Vector3d& rot1,
				        Eigen::MatrixXd& points2, ProcessT& gp2, Eigen::Vector3d& t2, Eigen::Vector3d& rot2,
                        bool norot, bool visualize)
{
    VisCallback* vis;
    if (visualize) {
        vis = new VisCallback(points1, points2, gp1, gp2, t1, rot1, t2, rot2);
    }
    Eigen::Matrix3d R1 = euler_to_matrix(rot1(0), rot1(1), rot1(2));
    Eigen::Matrix3d R2 = euler_to_matrix(rot2(0), rot2(1), rot2(2));

    double eps = 1.;
    int counter = 0;

    Eigen::VectorXd delta1(6);
	delta1.setZero();
    bool delta_diff_small = false;
    while (!delta_diff_small && counter < 200) {
		Eigen::VectorXd delta_old = delta1;
		cout << "Computing registration delta" << endl;
		delta1 = -compute_step(points2, gp1, t1, rot1, t2, rot2);
        if (norot) {
            delta1(2) = delta1(3) = delta1(4) = 0.;
        }
        delta_diff_small = (delta1 - delta_old).norm() < 1e-3f;
		cout << "Computing registration update" << endl;
		cout << "Registration update: " << delta1 << endl;
		cout << "Delta diff: " << (delta1 - delta_old).norm() << endl;
        t1.array() += eps*delta1.head<3>().array();
        rot1.array() += eps*0.00002*delta1.tail<3>().array();
        R1 = euler_to_matrix(rot1(0), rot1(1), rot1(2)); // add to total rotation
        if (delta_diff_small) {
            cout << "Done registering, exiting!" << endl;
        }
        if (visualize) {
            vis->visualizer_step(R1);
        }
        ++counter;
    }
}

int main(int argc, char** argv)
{
    string file_str;
    int first = -1;
    int second = -1;
    int subsample = 1;
    bool norot = false;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("file", "Input file", cxxopts::value(file_str))
      ("first", "First submap index", cxxopts::value(first))
      ("second", "Second submap index", cxxopts::value(second))
      ("norot", "No rotation?", cxxopts::value(norot))
      ("subsample", "Subsampling", cxxopts::value(subsample));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("file") == 0) {
		cout << "Please provide input file arg..." << endl;
		exit(0);
    }
	
    boost::filesystem::path path(file_str);
	cout << "Input file : " << path << endl;
    
    gp_submaps ss = read_data<gp_submaps>(path);
    ss.matches = get_medgaz_matches();
	//ss.trans[first].head<2>().array() += -5.0;
    //ss.angles[first](2) += 0.2;
    for (Eigen::MatrixXd& points : ss.points) {
        subsample_cloud(points, subsample);
    }
    pt_submaps::TransT trans_0 = ss.trans;
    pt_submaps::RotsT rots_0 = ss.rots;

    if (first == -1 || second == -1) {
        boost::filesystem::path track_benchmark_path(ss.dataset_name + "_benchmark.cereal");
        benchmark::track_error_benchmark track_benchmark = read_data<benchmark::track_error_benchmark>(track_benchmark_path);

        benchmark::registration_summary_benchmark benchmark(ss.dataset_name);
        for (const pair<int, int>& match : ss.matches) {
            first = match.first; second = match.second;
            cout << "Registering " << first << " and " << second << " ..." << endl;
            register_processes(ss.points[first], ss.gps[first], ss.trans[first], ss.angles[first],
                               ss.points[second], ss.gps[second], ss.trans[second], ss.angles[second],
                               norot, false);
            cout << "Done registering " << first << " and " << second << " ..." << endl;

            pt_submaps::TransT trans_corr;
            pt_submaps::RotsT rots_corr;
            Eigen::Matrix3d R_first = euler_to_matrix(ss.angles[first][0], ss.angles[first][1], ss.angles[first][2]);
            Eigen::Matrix3d Rc_first = R_first*rots_0[first].transpose();
            Eigen::Vector3d tc_first = track_benchmark.submap_origin + ss.trans[first] - Rc_first*(track_benchmark.submap_origin + trans_0[first]);
            Eigen::Matrix3d R_second = euler_to_matrix(ss.angles[second][0], ss.angles[second][1], ss.angles[second][2]);
            Eigen::Matrix3d Rc_second = R_second*rots_0[second].transpose();
            Eigen::Vector3d tc_second = track_benchmark.submap_origin + ss.trans[second] - Rc_second*(track_benchmark.submap_origin + trans_0[second]);
            trans_corr.push_back(tc_first); trans_corr.push_back(tc_second);
            rots_corr.push_back(Rc_first); rots_corr.push_back(Rc_second);

            mbes_ping::PingsT pings_pair = benchmark::registration_summary_benchmark::get_submap_pings_pair(track_benchmark.input_pings, first, second);
            benchmark.add_registration_benchmark(pings_pair, trans_corr, rots_corr, first, second);
        }
        benchmark.print_summary();
        boost::filesystem::path benchmark_path(ss.dataset_name + "_registration_benchmark.cereal");
        write_data(benchmark, benchmark_path);
    }
    else {
        register_processes(ss.points[first], ss.gps[first], ss.trans[first], ss.angles[first],
                           ss.points[second], ss.gps[second], ss.trans[second], ss.angles[second],
                           norot, true);
    }

    return 0;
}
