#include <fstream>
#include <iostream>
#include <string>
#include <cxxopts.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <ceres/ceres.h>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/colormap.h>
#include <data_tools/submaps.h>

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>

using namespace std;
using TransT = vector<vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >;
using RotsT = vector<vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > >;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using SubmapsGPT = vector<vector<ProcessT> >; // Process does not require aligned allocation as all matrices are dynamic
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

tuple<SubmapsGPT, TransT, RotsT> train_gps(SubmapsT& submaps, double lsq, double sigma, double s0)
{
    //if (boost::filesystem::exists("gp_submaps.json")) {
    if (boost::filesystem::exists("gp_submaps.cereal")) {
		TransT trans;
		RotsT rots;
		SubmapsGPT gps;
        //std::ifstream is("gp_submaps.json");
        std::ifstream is("gp_submaps.cereal", std::ifstream::binary);
        {
			//cereal::JSONInputArchive archive(is);
			cereal::BinaryInputArchive archive(is);
			//archive(gps, trans, rots);
			archive(trans, rots, gps);
        }
        is.close();
		return make_tuple(gps, trans, rots);
    }

    // check if already available
    TransT trans(submaps.size());
	RotsT rots(submaps.size());
	SubmapsGPT gps(submaps.size());
    for (int i = 0; i < submaps.size(); ++i) {
		trans[i].resize(submaps[i].size());
		rots[i].resize(submaps[i].size());
        for (int j = 0; j < submaps[i].size(); ++j) {
			ProcessT gp(100, s0);
			gp.kernel.sigmaf_sq = sigma;
			gp.kernel.l_sq = lsq*lsq;
			gp.kernel.p(0) = gp.kernel.sigmaf_sq;
			gp.kernel.p(1) = gp.kernel.l_sq;
			//tie(trans[i][j], rots[i][j]) = train_gp(submaps[i][j], gp);
			Eigen::Matrix3d R;
			Eigen::Vector3d t;
            // this will also centralize the points
			tie(t, R) = train_gp(submaps[i][j], gp);
			cout << "Main vector size: " << gps.size() << " and i is: " << i << endl;
			cout << "Done training gp, pushing back to vector of size: " << gps[i].size() << endl;
			trans[i][j] = t;
			rots[i][j] = R;
			gps[i].push_back(gp);
			cout << "Pushed back..." << endl;
        }
    }
	// write to disk for next time
    //std::ofstream os("gp_submaps.json");
    std::ofstream os("gp_submaps.cereal", std::ofstream::binary);
	{
		//cereal::JSONOutputArchive archive(os);
		cereal::BinaryOutputArchive archive(os);
		//archive(gps, trans, rots);
		archive(trans, rots, gps);
	}
    os.close();
    
    std::ofstream oss("submaps.cereal", std::ofstream::binary);
	{
		//cereal::JSONOutputArchive archive(os);
		cereal::BinaryOutputArchive archive(oss);
		//archive(gps, trans, rots);
		archive(submaps);
	}
    oss.close();
    
	return make_tuple(gps, trans, rots);
}

tuple<TransT, RotsT> distort_transforms(const TransT& trans, const RotsT& rots)
{
    TransT new_trans = trans;
	RotsT new_rots = rots;
	return make_tuple(new_trans, new_rots);
}

CloudT::Ptr construct_submap_and_gp_cloud(Eigen::MatrixXd points, ProcessT& gp,
				                          Eigen::Vector3d& t, Eigen::Matrix3d& R,
										  int offset)
{
	/*double meanx = points.col(0).mean();
	double meany = points.col(1).mean();
	double meanz = points.col(2).mean();
	Eigen::Vector3f mean_vector(meanx, meany, meanz);
	
	points.col(0).array() -= meanx;
	points.col(1).array() -= meany;
	points.col(2).array() -= meanz;*/
    
	/*for (int i = 0; i < points.rows(); ++i) {
		if (points(i, 2) < -10.) {
            points(i, 2) = -10.;
		}
    }*/

	double maxx = points.col(0).maxCoeff();
	double minx = points.col(0).minCoeff();
	double maxy = points.col(1).maxCoeff();
	double miny = points.col(1).minCoeff();
	double maxz = points.col(2).maxCoeff();
	double minz = points.col(2).minCoeff();

	cout << "Max z: " << maxz << ", Min z: " << minz << endl;

	int sz = 50;
	double xstep = (maxx - minx)/float(sz-1);
	double ystep = (maxy - miny)/float(sz-1);
    
	cout << "Predicting gaussian process..." << endl;

	Eigen::MatrixXd X_star(sz*sz, 2);
    Eigen::VectorXd f_star(sz*sz); // mean?
    f_star.setZero();
	Eigen::VectorXd V_star; // variance?
    int i = 0;
    for (int y = 0; y < sz; ++y) { // ROOM FOR SPEEDUP
	    for (int x = 0; x < sz; ++x) {
		    //if (i % 100 != 0) {
            //    continue;
            //}
		    //int ind = x*sz + y;
		    /*if (!W(ind, i)) {
			    continue;
		    }*/
		    X_star(i, 0) = minx + x*xstep;
		    X_star(i, 1) = miny + y*ystep;
		    ++i;
	    }
    }
    //X_star.conservativeResize(i, 2);
    gp.predict_measurements(f_star, X_star, V_star);
	//cout << "Predicted heights: " << f_star << endl;
	
	cout << "Done predicting gaussian process..." << endl;
	cout << "X size: " << maxx - minx << endl;
	cout << "Y size: " << maxy - miny << endl;
	cout << "Z size: " << maxz - minz << endl;

	Eigen::MatrixXd predicted_points(X_star.rows(), 3);
	predicted_points.leftCols(2) = X_star;
	predicted_points.col(2) = 1.*f_star;

	CloudT::Ptr cloud(new CloudT);

	predicted_points *= R.transpose();

    for (int i = 0; i < predicted_points.rows(); ++i) {
        PointT p;
        p.getVector3fMap() = predicted_points.row(i).cast<float>().transpose() + t.cast<float>();
        p.r = colormap[offset][0];
	    p.g = colormap[offset][1];
		p.b = colormap[offset][2];
		cloud->push_back(p);
    }

    points *= R.transpose();

    for (int i = 0; i < points.rows(); ++i) {
        PointT p;
        p.getVector3fMap() = points.row(i).cast<float>().transpose() + t.cast<float>();
        p.r = colormap[offset+1][0];
	    p.g = colormap[offset+1][1];
		p.b = colormap[offset+1][2];
		cloud->push_back(p);
    }

	return cloud;
}

SubmapsT load_submaps(const boost::filesystem::path& folder)
{
    SubmapsT submaps;
    if (boost::filesystem::exists("submaps.cereal")) {
        std::ifstream is("submaps.cereal", std::ifstream::binary);
        {
			//cereal::JSONInputArchive archive(is);
			cereal::BinaryInputArchive archive(is);
			//archive(gps, trans, rots);
			archive(submaps);
        }
        is.close();
    }
    else {
        submaps = read_submaps(folder);
    }
    return submaps;
}

void visualize_cloud(CloudT::Ptr& cloud)
{
	cout << "Done constructing point cloud, starting viewer..." << endl;

	//... populate cloud
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ())
	{
	}
}

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

	SubmapsGPT submap_gps;
	TransT trans;
	RotsT rots;

    SubmapsT submaps = load_submaps(folder);

	// NOTE: this function checks if the file submap_gps.json is available
	// If it is, it will use the already trained gps. If you're using new
	// parameters, delete that file
	tie(submap_gps, trans, rots) = train_gps(submaps, lsq, sigma, s0);

	tie(trans, rots) = distort_transforms(trans, rots);
    
	Eigen::MatrixXd points3 = get_points_in_bound_transform(submaps[0][1], trans[0][1], rots[0][1], trans[0][0], rots[0][0], 465);
    CloudT::Ptr cloud(new CloudT);
    for (int i = 0; i < submaps.size(); ++i) {
        for (int j = 0; j < submaps[i].size(); ++j) {
	        CloudT::Ptr subcloud = construct_submap_and_gp_cloud(submaps[i][j], submap_gps[i][j], trans[i][j], rots[i][j], 2*(i*submaps[i].size() + j));
            *cloud += *subcloud;
        }
    }
	
    //CloudT::Ptr cloud1 = construct_submap_and_gp_cloud(submaps[0][0], submap_gps[0][0], trans[0][0], rots[0][0], 0);
	//CloudT::Ptr cloud2 = construct_submap_and_gp_cloud(submaps[0][1], submap_gps[0][1], trans[0][1], rots[0][1], 2);
	//CloudT::Ptr cloud3 = construct_cloud(points3, trans[0][0], rots[0][0], 4);

	//*cloud1 += *cloud2;
	//*cloud1 += *cloud3;
	visualize_cloud(cloud);

	return 0;
}
