#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOFF.h>

#include <Eigen/Dense>
#include <cxxopts.hpp>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/probit_noise.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/colormap.h>
#include <data_tools/submaps.h>

using namespace std;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;

void visualize_submap_and_gp(Eigen::MatrixXd& points, ProcessT& gp)
{
	double meanx = points.col(0).mean();
	double meany = points.col(1).mean();
	double meanz = points.col(2).mean();
	
	points.col(0).array() -= meanx;
	points.col(1).array() -= meany;
	points.col(2).array() -= meanz;
    
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
    int nbr_faces = 2*(sz-1)*(sz-1);
    Eigen::MatrixXi F(nbr_faces, 3);
    int face_counter = 0;
    for (int y = 0; y < sz; ++y) { // ROOM FOR SPEEDUP
	    for (int x = 0; x < sz; ++x) {
		    X_star(y*sz+x, 0) = minx + x*xstep;
		    X_star(y*sz+x, 1) = miny + y*ystep;
            if (x > 0 && y > 0) {
                F.row(face_counter) << y*sz+x, y*sz+x-1, (y-1)*sz+x;
                ++face_counter;
            }
            if (x < sz-1 && y < sz-1) {
                F.row(face_counter) << y*sz+x, y*sz+x+1, (y+1)*sz+x;
                ++face_counter;
            }
	    }
    }

    cout << "F size: " << F.rows() << ", face count: " << face_counter << endl;

    gp.predict_measurements(f_star, X_star, V_star);
	cout << "Predicted heights: " << f_star << endl;
	
	cout << "Done predicting gaussian process..." << endl;
	cout << "X size: " << maxx - minx << endl;
	cout << "Y size: " << maxy - miny << endl;
	cout << "Z size: " << maxz - minz << endl;

	Eigen::MatrixXd V(X_star.rows(), 3);
	V.leftCols(2) = X_star;
	V.col(2) = f_star;

    // Load a mesh in OFF format
    //igl::readOFF("bunny.off", V, F);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);

    // Use the z coordinate as a scalar field over the surface
    Eigen::VectorXd Z = V.col(2);

    // Compute per-vertex colors
    Eigen::MatrixXd C;
    igl::jet(Z, true, C);

    // Add per-vertex colors
    viewer.data().set_colors(C);

    viewer.launch();
}

void train_gp(Eigen::MatrixXd& points, ProcessT& gp)
{
    cout << "Training gaussian process..." << endl;
	double meanx = points.col(0).mean();
	double meany = points.col(1).mean();
	double meanz = points.col(2).mean();

    Eigen::MatrixXd X = points.leftCols(2);
	X.col(0).array() -= meanx;
	X.col(1).array() -= meany;
	Eigen::VectorXd y = points.col(2).array() - meanz;
	//gp.train_parameters(X, y);
	gp.add_measurements(X, y);

    cout << "Done training gaussian process..." << endl;
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
      ("s0", "Probit noise", cxxopts::value(s0));

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
	
	Eigen::MatrixXd points = read_submap(folder / "patch_00_00.xyz");
	
	ProcessT gp(100, s0);
	gp.kernel.sigmaf_sq = sigma;
	gp.kernel.l_sq = lsq*lsq;
    gp.kernel.p(0) = gp.kernel.sigmaf_sq;
    gp.kernel.p(1) = gp.kernel.l_sq;
	train_gp(points, gp);

	visualize_submap_and_gp(points, gp);

    return 0;
}
