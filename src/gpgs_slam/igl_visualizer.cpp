#include <gpgs_slam/igl_visualizer.h>
#include <gpgs_slam/transforms.h>
#include <data_tools/submaps.h>
#include <data_tools/colormap.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <gpgs_slam/visualization.h>

#include <igl/opengl/gl.h>

using namespace std;

tuple<Eigen::MatrixXd, Eigen::MatrixXi> IglVisCallback::vertices_faces_from_gp(Eigen::MatrixXd& points, ProcessT& gp)
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

	double xstep = (maxx - minx)/float(sz-1);
	double ystep = (maxy - miny)/float(sz-1);
    
	cout << "Predicting gaussian process..." << endl;

    Eigen::MatrixXd X_star(sz*sz, 2);
    Eigen::VectorXd f_star(sz*sz); // mean?
    f_star.setZero();
	Eigen::VectorXd V_star; // variance?
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
	
	cout << "Done predicting gaussian process..." << endl;
	cout << "X size: " << maxx - minx << endl;
	cout << "Y size: " << maxy - miny << endl;
	cout << "Z size: " << maxz - minz << endl;

	Eigen::MatrixXd V(X_star.rows(), 3);
	V.leftCols(2) = X_star;
	V.col(2) = f_star;

	return make_tuple(V, F);
}

IglVisCallback::IglVisCallback(ObsT& points, SubmapsGPT& gps, TransT& trans, AngsT& rots)
    :  points(points), gps(gps), trans(trans), rots(rots)
{
	updated = false;
	sz = 50;
    nbr_faces = 2*(sz-1)*(sz-1);
	nbr_vertices = sz*sz;

	int nbr_processes = points.size();

	V_orig.resize(nbr_processes*nbr_vertices, 3);
	V.resize(nbr_processes*nbr_vertices, 3);
	V_new.resize(nbr_processes*nbr_vertices, 3);
	F.resize(nbr_processes*nbr_faces, 3);
	C.resize(nbr_processes*nbr_faces, 3);

    for (int i = 0; i < points.size(); ++i) {
        Eigen::Matrix3d RM = euler_to_matrix(rots[i](0), rots[i](1), rots[i](2));
        //CloudT::Ptr cloud = construct_submap_and_gp_cloud(points[i], gps[i], trans[i], RM, 2*i);
        //viewer.showCloud(cloud, string("cloud")+to_string(i));
		Eigen::MatrixXd Vi;
		Eigen::MatrixXi Fi;
		tie(Vi, Fi) = vertices_faces_from_gp(points[i], gps[i]);
		V_orig.block(i*nbr_vertices, 0, nbr_vertices, 3) = Vi;
		F.block(i*nbr_faces, 0, nbr_faces, 3) = Fi.array() + i*nbr_vertices;
		Eigen::RowVector3d Ci(double(colormap[i%43][0])/255., double(colormap[i%43][1])/255., double(colormap[i%43][2])/255.);
		C.block(i*nbr_faces, 0, nbr_faces, 3).rowwise() = Ci;
		V_new.block(i*nbr_vertices, 0, nbr_vertices, 3) = (V_orig.block(i*nbr_vertices, 0, nbr_vertices, 3)*RM.transpose()).rowwise() + trans[i].transpose();
    }
	V = V_new;

    /*
    vis = visualize_likelihoods(t2, RM2);
    t0 = t1;
    old_point = cv::Point(vis.cols/2+0, vis.rows/2+0);

    step_offset = 15.;
    factor = 20.;
    */
	viewer.data().set_mesh(V, F);
    // Add per-vertex colors
    viewer.data().set_colors(C);

    viewer.callback_pre_draw = std::bind(&IglVisCallback::callback_pre_draw, this, std::placeholders::_1);
    viewer.core.is_animating = true;
    viewer.core.animation_max_fps = 30.;
	//viewer.launch();

    vis = cv::imread("temp.png");
    cv::imshow("registration", vis);
    cv::waitKey(0);
}

void IglVisCallback::display()
{
	viewer.launch();
}

bool IglVisCallback::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    glEnable(GL_CULL_FACE);

    if (viewer.core.is_animating && updated) {
		V = V_new;
        viewer.data().set_vertices(V);
        viewer.data().compute_normals();
		updated = false;
    }

    return false;
}

void IglVisCallback::visualizer_step(vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >& RMs)
{
    /*
    Vector3d rt = t1 - t0;
    cv::Point new_point(vis.cols/2+int(factor*(rt(0)/step_offset+0.5)), vis.rows/2+int(factor*(rt(1)/step_offset+0.5)));
    cv::line(vis, old_point, new_point, cv::Scalar(0, 0, 255)); //, int thickness=1, int lineType=8, int shift=0)
    old_point = new_point;
    //cv::waitKey(10);
    */
    cout << "Visualizing step" << endl;
    for (int i = 0; i < points.size(); ++i) {
		V_new.block(i*nbr_vertices, 0, nbr_vertices, 3) = (V_orig.block(i*nbr_vertices, 0, nbr_vertices, 3)*RMs[i].transpose()).rowwise() + trans[i].transpose();
    }
	updated = true;

    //Eigen::MatrixXd points3 = get_points_in_bound_transform(points2, t2, R2, t1, R1, 465);
    /*for (int i = 0; i < points.size(); ++i) {
        CloudT::Ptr cloud = construct_submap_and_gp_cloud(points[i], gps[i], trans[i], RMs[i], 2*i);
        viewer.removeVisualizationCallable(string("cloud")+to_string(i));
        viewer.showCloud(cloud, string("cloud")+to_string(i));
    }*/
    
    //cv::imshow("registration", vis);
    //cv::waitKey(0);
}

ceres::CallbackReturnType IglVisCallback::operator()(const ceres::IterationSummary& summary)
{
    vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > RMs;
    for (const Eigen::Vector3d& rot : rots) {
        RMs.push_back(euler_to_matrix(rot(0), rot(1), rot(2)));
    }
    visualizer_step(RMs);
    return ceres::SOLVER_CONTINUE;
}
