#include <Eigen/Dense>
#include <cxxopts.hpp>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>

#include <gpgs_slam/transforms.h>

#include <data_tools/submaps.h>
#include <data_tools/data_structures.h>

#include <random>

using namespace std;

gp_submaps parse_and_train_gps(double lsq, double sigma, double s0, const boost::filesystem::path& folder)
{

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
    for (int i = 0; i < ss.points.size(); ++i) {
        gp_submaps::ProcessT gp(100, s0);
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
    
    return ss;
}

int main(int argc, char** argv)
{
    string folder_str;
    string file_str;
	double lsq = 100.;
	double sigma = 10.;
	double s0 = 1.;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("folder", "Input folder", cxxopts::value(folder_str))
      ("file", "Output file", cxxopts::value(file_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("s0", "Measurement noise", cxxopts::value(s0));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("folder") == 0) {
		cout << "Please provide input folder arg..." << endl;
		exit(0);
    }
    if (result.count("file") == 0) {
		cout << "Please provide output file arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path folder(folder_str);
    boost::filesystem::path path(file_str);

	cout << "Input folder : " << folder << endl;
	cout << "Output file : " << path << endl;

    gp_submaps ss = parse_and_train_gps(lsq, sigma, s0, folder);
	
    write_data(ss, path);

    return 0;
}
