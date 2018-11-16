#include <bathy_maps/mesh_map.h>
#include <sonar_tracing/snell_ray_tracing.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>
#include <igl/readSTL.h>
#include <igl/ray_mesh_intersect.h>
#include <igl/embree/line_mesh_intersection.h>
#include <igl/unproject_onto_mesh.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

using namespace std;

std::tuple<uint8_t, uint8_t, uint8_t> jet_mesh(double x)
{
    const double rone = 0.8;
    const double gone = 1.0;
    const double bone = 1.0;
    double r, g, b;

    x = (x < 0 ? 0 : (x > 1 ? 1 : x));

    if (x < 1. / 8.) {
        r = 0;
        g = 0;
        b = bone * (0.5 + (x) / (1. / 8.) * 0.5);
    } else if (x < 3. / 8.) {
        r = 0;
        g = gone * (x - 1. / 8.) / (3. / 8. - 1. / 8.);
        b = bone;
    } else if (x < 5. / 8.) {
        r = rone * (x - 3. / 8.) / (5. / 8. - 3. / 8.);
        g = gone;
        b = (bone - (x - 3. / 8.) / (5. / 8. - 3. / 8.));
    } else if (x < 7. / 8.) {
        r = rone;
        g = (gone - (x - 5. / 8.) / (7. / 8. - 5. / 8.));
        b = 0;
    } else {
        r = (rone - (x - 7. / 8.) / (1. - 7. / 8.) * 0.5);
        g = 0;
        b = 0;
    }

    return std::make_tuple(uint8_t(255.*r), uint8_t(255.*g), uint8_t(255.*b));
}

void bathy_map_mesh::display_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);
    // Add per-vertex colors
    //viewer.data().set_colors(C);
    Eigen::MatrixXd C_jet;
    igl::jet(V.col(2), true, C_jet);
    // Add per-vertex colors
    viewer.data().set_colors(C_jet);

    viewer.data().point_size = 10;
    viewer.data().line_width = 1;

    //viewer.callback_pre_draw = std::bind(&IglVisCallback::callback_pre_draw, this, std::placeholders::_1);
    //viewer.callback_key_pressed = std::bind(&IglVisCallback::callback_key_pressed, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    viewer.core.is_animating = true;
    viewer.core.animation_max_fps = 30.;
	//viewer.launch();
    viewer.core.background_color << 1., 1., 1., 1.; // white background
	viewer.launch();
}

void bathy_map_mesh::display_height_map(const Eigen::MatrixXd& height_map)
{
    double minv = height_map.minCoeff();
    Eigen::ArrayXXd height_map_array = height_map.array();
    height_map_array -= minv*(height_map_array > 0).cast<double>();
    double maxv = height_map_array.maxCoeff();
    height_map_array /= maxv;

    int rows = height_map.rows();
    int cols = height_map.cols();
    cv::Mat bathy_map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (height_map_array(i, j) == 0) {
                continue;
            }
            cv::Point3_<uchar>* p = bathy_map.ptr<cv::Point3_<uchar> >(rows-i-1, j);
            tie(p->z, p->y, p->x) = jet_mesh(height_map_array(i, j));
        }
    }

    cv::imshow("My image", bathy_map);
    cv::waitKey();
}

pair<Eigen::MatrixXd, bathy_map_mesh::BoundsT> bathy_map_mesh::height_map_from_pings(const mbes_ping::PingsT& pings, double res)
{
    auto xcomp = [](const mbes_ping& p1, const mbes_ping& p2) {
        return p1.pos_[0] < p2.pos_[0];
    };
    auto ycomp = [](const mbes_ping& p1, const mbes_ping& p2) {
        return p1.pos_[1] < p2.pos_[1];
    };
    double maxx = std::max_element(pings.begin(), pings.end(), xcomp)->pos_[0];
    double minx = std::min_element(pings.begin(), pings.end(), xcomp)->pos_[0];
    double maxy = std::max_element(pings.begin(), pings.end(), ycomp)->pos_[1];
    double miny = std::min_element(pings.begin(), pings.end(), ycomp)->pos_[1];

    cout << "Min X: " << minx << ", Max X: " << maxx << ", Min Y: " << miny << ", Max Y: " << maxy << endl;

    int cols = std::ceil((maxx-minx)/res);
    int rows = std::ceil((maxy-miny)/res);
    maxx = minx + double(cols)*res;
    maxy = miny + double(rows)*res;

    cout << "Target res: " << res << endl;
    cout << "Initial cols: " << cols << ", rows: " << rows << endl;

    Eigen::MatrixXd means(rows, cols); means.setZero();
    Eigen::MatrixXd counts(rows, cols); counts.setZero();

    for (const mbes_ping& ping : pings) {
        for (const Eigen::Vector3d& pos : ping.beams) {
            int col = int((pos[0]-minx)/res);
            int row = int((pos[1]-miny)/res);
            if (col >= 0 && col < cols && row >= 0 && row < rows) {
                means(row, col) += pos[2];
                counts(row, col) += 1.;
            }
        }
    }

    Eigen::ArrayXXd counts_pos = counts.array() + (counts.array() == 0.).cast<double>();
    means.array() /= counts_pos;

    BoundsT bounds; bounds << minx, miny, maxx, maxy;

    return make_pair(means, bounds);
}

pair<Eigen::MatrixXd, Eigen::MatrixXi> bathy_map_mesh::mesh_from_height_map(const Eigen::MatrixXd& height_map, const BoundsT& bounds)
{
    // these are the bottom-left corners and top-right corners of height map respectively
	double maxx = bounds(1, 0);
	double minx = bounds(0, 0);
	double maxy = bounds(1, 1);
	double miny = bounds(0, 1);

    int rows = height_map.rows();
    int cols = height_map.cols();
	double xres = (maxx - minx)/double(cols);
	double yres = (maxy - miny)/double(rows);

    cout << "X res: " << xres << ", Y res: " << yres << endl;
    double res = xres;

    int nbr_faces = 2*(rows-1)*(cols-1);
    Eigen::MatrixXd V(rows*cols, 3);
    Eigen::MatrixXi F(nbr_faces, 3);
    int face_counter = 0;
    for (int y = 0; y < rows; ++y) { // ROOM FOR SPEEDUP
	    for (int x = 0; x < cols; ++x) {
            //V.row(y*cols+x) << minx+(double(x)+.5)*res, miny+(double(y)+.5)*res, height_map(y, x);
            V.row(y*cols+x) << (double(x)+.5)*res, (double(y)+.5)*res, height_map(y, x);
            if (height_map(y, x) == 0) {
                continue;
            }
            if (x > 0 && y > 0 && height_map(y, x-1) != 0 && height_map(y-1, x) != 0) {
                F.row(face_counter) << y*cols+x, y*cols+x-1, (y-1)*cols+x;
                ++face_counter;
            }
            if (x < cols-1 && y < rows-1 && height_map(y+1, x) != 0 && height_map(y, x+1) != 0) {
                F.row(face_counter) << y*cols+x, y*cols+x+1, (y+1)*cols+x;
                ++face_counter;
            }
	    }
    }

    cout << "F size: " << F.rows() << ", face count: " << face_counter << endl;
    F.conservativeResize(face_counter, 3);
	cout << "X size: " << maxx - minx << endl;
	cout << "Y size: " << maxy - miny << endl;

    return make_pair(V, F);
}

tuple<Eigen::MatrixXd, Eigen::MatrixXi, bathy_map_mesh::BoundsT> bathy_map_mesh::mesh_from_pings(const mbes_ping::PingsT& pings, double res)
{
    Eigen::MatrixXd height_map;
    BoundsT bounds;
    tie(height_map, bounds) = height_map_from_pings(pings, res);
    display_height_map(height_map);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    tie(V, F) = mesh_from_height_map(height_map, bounds);
    return make_tuple(V, F, bounds);
}

struct survey_viewer {
    igl::opengl::glfw::Viewer viewer;
    const xtf_sss_ping::PingsT& pings;
    int i;
    Eigen::MatrixXd V1;
    Eigen::MatrixXi F1;
    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    Eigen::Vector3d offset;
    Eigen::VectorXd hit_sums;
    Eigen::VectorXi hit_counts;
    Eigen::MatrixXd N_faces; // the normals of F1, V1

    survey_viewer(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
        const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
        const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset)
        : pings(pings), i(0), V1(V1), F1(F1), V2(V2), F2(F2), offset(offset)
    {
        //double first_heading = pings[0].heading_;
        hit_sums = Eigen::VectorXd(V1.rows()); hit_sums.setZero();
        hit_counts = Eigen::VectorXi(V1.rows()); hit_counts.setZero();

        V = Eigen::MatrixXd(V1.rows() + V2.rows(), V1.cols());
        V << V1, this->V2;
        F = Eigen::MatrixXi(F1.rows() + F2.rows(), F1.cols());
        F << F1, (F2.array() + V1.rows());

        V.bottomRows(V2.rows()) = V2;
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[0].heading_, Eigen::Vector3d::UnitZ()).matrix();
        V.bottomRows(V2.rows()) *= Rz.transpose();
        V.bottomRows(V2.rows()).array().rowwise() += (pings[0].pos_ - offset).transpose().array();

        C = Eigen::MatrixXd(C1.rows()+C2.rows(), C1.cols());
        C << C1, C2;

        // Compute per-face normals
        igl::per_face_normals(V1, F1, N_faces);

        viewer.data().set_mesh(V, F);
        // Add per-vertex colors
        //viewer.data().set_colors(C);
        // Add per-vertex colors
        viewer.data().set_colors(C);

        viewer.data().point_size = 10;
        viewer.data().line_width = 1;

        //viewer.callback_pre_draw = std::bind(&survey_viewer::callback_pre_draw, this, std::placeholders::_1);
        viewer.callback_mouse_down = std::bind(&survey_viewer::callback_mouse_down, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        viewer.callback_key_pressed = std::bind(&survey_viewer::callback_key_pressed, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        viewer.core.is_animating = true;
        viewer.core.animation_max_fps = 30.;
        //viewer.launch();
        viewer.core.background_color << 1., 1., 1., 1.; // white background
    }

    void launch()
    {
        viewer.launch();
    }
    
    pair<Eigen::MatrixXd, Eigen::MatrixXd> compute_sss_dirs(const Eigen::Matrix3d& R, double tilt_angle, double beam_width)
    {
        const double min_theta = tilt_angle - 0.5*beam_width; // M_PI/180.*10.;
        const double max_theta = tilt_angle + 0.5*beam_width; //M_PI/180.*60.;
        const int nbr_lines = 200;

        double min_c = 1./cos(min_theta);
        double max_c = 1./cos(max_theta);
        double step = (max_c - min_c)/double(nbr_lines-1);

        Eigen::MatrixXd dirs_left(nbr_lines, 3);
        Eigen::MatrixXd dirs_right(nbr_lines, 3);
        for (int i = 0; i < nbr_lines; ++i) {
            double ci = min_c + double(i)*step;
            double bi = sqrt(ci*ci-1.);
            Eigen::Vector3d dir_left(0., bi, -1.);
            Eigen::Vector3d dir_right(0., -bi, -1.);
            dirs_left.row(i) = (R*dir_left).transpose();
            dirs_right.row(i) = (R*dir_right).transpose();
        }

        return make_pair(dirs_left, dirs_right);
    }

    tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi> compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width)
    {
        igl::Hit hit;
        Eigen::MatrixXd dirs_left;
        Eigen::MatrixXd dirs_right;
        tie(dirs_left, dirs_right) = compute_sss_dirs(R, tilt_angle, beam_width);
        Eigen::MatrixXd hits_left(dirs_left.rows(), 3);
        Eigen::MatrixXd hits_right(dirs_right.rows(), 3);
        Eigen::VectorXi hits_left_inds(dirs_left.rows());
        Eigen::VectorXi hits_right_inds(dirs_right.rows());
        int hit_count = 0;
        for (int i = 0; i < dirs_left.rows(); ++i) {
            bool did_hit = ray_mesh_intersect(origin, dirs_left.row(i).transpose(), V1, F1, hit);
            if (did_hit) {
                int vind = F1(hit.id, 0);
                hits_left_inds(hit_count) = hit.id;
                // we actually get the coordinates within the triangle also, we could use that
                hits_left.row(hit_count) = V1.row(vind);
                ++hit_count;
            }
        }
        hits_left.conservativeResize(hit_count, 3);
        hits_left_inds.conservativeResize(hit_count);
        hit_count = 0;
        for (int i = 0; i < dirs_right.rows(); ++i) {
            bool did_hit = ray_mesh_intersect(origin, dirs_right.row(i).transpose(), V1, F1, hit);
            if (did_hit) {
                int vind = F1(hit.id, 0);
                hits_right_inds(hit_count) = hit.id;
                // we actually get the coordinates within the triangle also, we could use that
                hits_right.row(hit_count) = V1.row(vind);
                ++hit_count;
            }
        }
        hits_right.conservativeResize(hit_count, 3);
        hits_right_inds.conservativeResize(hit_count);
        return make_tuple(hits_left, hits_right, hits_left_inds, hits_right_inds);
    }

    tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXd, Eigen::VectorXd> embree_compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width)
    {
        auto start = chrono::high_resolution_clock::now();
        igl::Hit hit;
        Eigen::MatrixXd dirs_left;
        Eigen::MatrixXd dirs_right;
        tie(dirs_left, dirs_right) = compute_sss_dirs(R, tilt_angle, beam_width);
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "compute_sss_dirs time: " << duration.count() << " microseconds" << endl;

        start = chrono::high_resolution_clock::now();
        Eigen::MatrixXd hits_left(dirs_left.rows(), 3);
        Eigen::MatrixXd hits_right(dirs_right.rows(), 3);
        Eigen::VectorXi hits_left_inds(dirs_left.rows());
        Eigen::VectorXi hits_right_inds(dirs_right.rows());
        Eigen::VectorXd mod_left(dirs_left.rows());
        Eigen::VectorXd mod_right(dirs_right.rows());

        int nbr_lines = dirs_left.rows() + dirs_right.rows();
        Eigen::MatrixXd dirs = Eigen::MatrixXd(nbr_lines, dirs_left.cols());
        dirs << dirs_left, dirs_right;
        Eigen::MatrixXd P = Eigen::MatrixXd(nbr_lines, 3);
        P.rowwise() = origin.transpose();
        Eigen::MatrixXd hits = igl::embree::line_mesh_intersection(P, dirs, V1, F1);
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "line_mesh_intersection time: " << duration.count() << " microseconds" << endl;

        start = chrono::high_resolution_clock::now();
        int hit_count = 0;
        for (int i = 0; i < dirs_left.rows(); ++i) {
            int hit = hits(i, 0);
            if (hit > 0) {
                int vind = F1(hit, 0);
                hits_left_inds(hit_count) = hit;
                // we actually get the coordinates within the triangle also, we could use that
                hits_left.row(hit_count) = V1.row(vind);
                double nn = (origin - hits_left.row(hit_count).transpose()).norm();
                Eigen::Vector3d dir = origin - hits_left.row(hit_count).transpose();
                dir.normalize();
                mod_left(hit_count) = 1.; //(1./dir.dot(N_faces.row(hit).transpose()))*(nn/60.);
                ++hit_count;
            }
        }
        hits_left.conservativeResize(hit_count, 3);
        hits_left_inds.conservativeResize(hit_count);
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "hits_left loop time: " << duration.count() << " microseconds" << endl;

        start = chrono::high_resolution_clock::now();
        hit_count = 0;
        for (int i = 0; i < dirs_right.rows(); ++i) {
            int hit = hits(dirs_left.rows() + i, 0);
            if (hit > 0) {
                int vind = F1(hit, 0);
                hits_right_inds(hit_count) = hit;
                // we actually get the coordinates within the triangle also, we could use that
                hits_right.row(hit_count) = V1.row(vind);
                double nn = (origin - hits_right.row(hit_count).transpose()).norm();
                Eigen::Vector3d dir = origin - hits_right.row(hit_count).transpose();
                dir.normalize();
                mod_right(hit_count) = 1.; //(1./dir.dot(N_faces.row(hit).transpose()))*(nn/60.);
                ++hit_count;
            }
        }
        hits_right.conservativeResize(hit_count, 3);
        hits_right_inds.conservativeResize(hit_count);
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "hits_right loop time: " << duration.count() << " microseconds" << endl;

        return make_tuple(hits_left, hits_right, hits_left_inds, hits_right_inds, mod_left, mod_right);
    }

    void correlate_hits(const Eigen::MatrixXd& hits_port,
                        const Eigen::VectorXi& hits_port_inds,
                        const Eigen::VectorXd& mod_port,
                        const xtf_sss_ping_side& ping,
                        const Eigen::Vector3d& origin,
                        double sound_vel)
    {

        Eigen::VectorXd layer_depths(4);
        layer_depths << -5., -10., -15., -20.;
        Eigen::VectorXd layer_speeds(5);
        layer_speeds << 1506.43, 1504.47, 1498.61, 1495.05, 1492.64;
        //layer_speeds << 1706.43, 1604.47, 1498.61, 1395.05, 1292.64;

        // we do not take roll into account but we do need to account for the pitch
        // if the vehicle has pitch, we need to extend the layer depths
        // ok, let's just take the norm of the first two coordinates to begin with
        // actually, that already fixes the pitch rotation
        
        //Eigen::Vector3d origin = ping.pos_ - offset;
        Eigen::VectorXd times_port_simple = 1.*(hits_port.rowwise() - origin.transpose()).rowwise().norm()/sound_vel; //ping.sound_vel_;

        Eigen::VectorXd times_port;
        if (false) {
            Eigen::VectorXd x = (hits_port.leftCols<2>().rowwise() - origin.head<2>().transpose()).rowwise().norm();
            Eigen::MatrixXd end_points(x.rows(), 2);
            end_points.col(0) = x;
            end_points.col(1) = hits_port.col(2);
        
            Eigen::MatrixXd layer_widths;
            tie(times_port, layer_widths) = trace_multiple_layers(layer_depths, layer_speeds, end_points);
            times_port.array() *= 2.; // back and forth
            cout << "Got final times: " << times_port.transpose() << endl;

            visualize_rays(end_points, layer_depths, layer_widths, -25.);
        }
        else {
            times_port = times_port_simple;
        }

        cout << "Compared to simple: " << times_port_simple.transpose() << endl;
        //Eigen::VectorXd times_stbd = 1.*(hits_stbd.rowwise() - origin.transpose()).rowwise().norm()/ping.sound_vel_;
        cout << "Port ping duration: " << ping.time_duration << endl;
        //cout << "Port times: " << times_port.transpose() << endl;
        //cout << "Stbd ping duration: " << ping.stbd.time_duration << endl;
        //cout << "Stbd times: " << times_stbd.transpose() << endl;
        

        if (times_port.rows() == 0) {
            return;
        }

        double port_step = ping.time_duration / double(ping.pings.size());
        cout << "port step: " << port_step << endl;
        int pos = 0;
        for (int i = 0; i < ping.pings.size(); ++i) {
            if (times_port(0) > double(i)*port_step) {
                continue;
            }
            if (pos >= hits_port_inds.rows()) {
                break;
            }

            //double intensity = (double(ping.port.pings[i]) + 32767.)/(2.*32767.);
            //double intensity = double(ping.pings[ping.pings.size()-i-1])/(10000.);
            /*if (intensity < 0.2) { // no hit?
                continue;
            }*/

            //cout << "Pos: " << pos << ", size: " << hits_port.rows() << endl;
            while (pos < hits_port.rows() && double(i)*port_step > times_port(pos)) {
                //cout << "Not good: " << double(i)*port_step << ", " << times_port(pos) << endl;
                ++pos;
            }
            if (pos >= hits_port.rows()) {
                break;
            }
            double intensity = mod_port(pos)*double(ping.pings[i])/(10000.);
            //cout << "Found one: " << double(i)*port_step << ", " << times_port(pos) << endl;
            //cout << "With intensity: " << intensity << endl;
            int vind = F1(hits_port_inds(pos), 0);
            hit_sums(vind) += intensity;
            hit_counts(vind) += 1;
            Eigen::Vector3d color = hit_sums(vind)/double(hit_counts(vind))*Eigen::Vector3d::Ones();
            C.row(vind) = color.transpose();
        }

    }

    void project_sss()
    {
        if (i >= pings.size()) {
            return;
        }
        cout << "Setting new position: " << pings[i].pos_.transpose() << endl;
        //viewer.data().compute_normals();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
        Eigen::Matrix3d R = Rz*Ry;

        Eigen::MatrixXd hits_left;
        Eigen::MatrixXd hits_right;
        Eigen::VectorXi hits_left_inds;
        Eigen::VectorXi hits_right_inds;
        Eigen::VectorXd mod_left;
        Eigen::VectorXd mod_right;
        //tie(hits_left, hits_right, hits_left_inds, hits_right_inds) = compute_hits(pings[i].pos_ - offset, R, pings[i].port.tilt_angle, pings[i].port.beam_width);
        auto start = chrono::high_resolution_clock::now();
        tie(hits_left, hits_right, hits_left_inds, hits_right_inds, mod_left, mod_right) = embree_compute_hits(pings[i].pos_ - offset, R, 1.4*pings[i].port.tilt_angle, pings[i].port.beam_width);
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "embree_compute_hits time: " << duration.count() << " microseconds" << endl;

        start = chrono::high_resolution_clock::now();
        correlate_hits(hits_left, hits_left_inds, mod_left, pings[i].port, pings[i].pos_ - offset, pings[i].sound_vel_);
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "left correlate_hits time: " << duration.count() << " microseconds" << endl;

        start = chrono::high_resolution_clock::now();
        correlate_hits(hits_right, hits_right_inds, mod_right, pings[i].stbd, pings[i].pos_ - offset, pings[i].sound_vel_);
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "right correlate_hits time: " << duration.count() << " microseconds" << endl;

        if (i % 10 == 0) {
            start = chrono::high_resolution_clock::now();
            V.bottomRows(V2.rows()) = V2;
            V.bottomRows(V2.rows()) *= R.transpose();
            V.bottomRows(V2.rows()).array().rowwise() += (pings[i].pos_ - offset).transpose().array();
            viewer.data().set_vertices(V);

            Eigen::MatrixXi E;
            Eigen::MatrixXd P(hits_left.rows(), 3);
            P.rowwise() = (pings[i].pos_ - offset).transpose();
            viewer.data().set_edges(P, E, Eigen::RowVector3d(1., 0., 0.));
            viewer.data().add_edges(P, hits_left, Eigen::RowVector3d(1., 0., 0.));
            P = Eigen::MatrixXd(hits_right.rows(), 3);
            P.rowwise() = (pings[i].pos_ - offset).transpose();
            viewer.data().add_edges(P, hits_right, Eigen::RowVector3d(0., 1., 0.));
            viewer.data().set_colors(C);
            stop = chrono::high_resolution_clock::now();
            duration = chrono::duration_cast<chrono::microseconds>(stop - start);
            cout << "vis time: " << duration.count() << " microseconds" << endl;
        }

    }

    bool point_in_view(const xtf_sss_ping& ping, const Eigen::Vector3d& point)
    {
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(ping.pitch_, Eigen::Vector3d::UnitY()).matrix();
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
        Eigen::Matrix3d R = Rz*Ry;

        // first, let's transform the point to a coordinate system defined by the sonar
        Eigen::Vector3d p = R.transpose()*(point - ping.pos_);

        // now, let's get the yaw and pitch components
        double yaw = atan2(p(1), p(0));

        double xy_dist = sqrt(p(1)*p(1)+p(0)*p(0));
        double pitch = atan(p(2)/xy_dist);

        // check if point is in view of either of the side scans
        bool yaw_in_view = fabs(yaw) < M_PI/2. + M_PI/8. && fabs(yaw) > M_PI/2. - M_PI/8.;

        bool pitch_in_view = pitch < 1.4*ping.port.tilt_angle + 0.5*ping.port.beam_width &&
                             pitch > 1.4*ping.port.tilt_angle - 0.5*ping.port.beam_width;

        return pitch_in_view && yaw_in_view;
    }

    bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
    {
        glEnable(GL_CULL_FACE);

        if (viewer.core.is_animating) {
            project_sss();
        }
    }

    bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int)
    {
        cout << "Got mouse callback!" << endl;
        int fid;
        Eigen::Vector3f bc;
        // Cast a ray in the view direction starting from the mouse position
        double x = viewer.current_mouse_x;
        double y = viewer.core.viewport(3) - viewer.current_mouse_y;
        if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view * viewer.core.model,
                viewer.core.proj, viewer.core.viewport, V1, F1, fid, bc)) {
            // paint hit red
            cout << "Got point in mesh!" << endl;
            int vind = F1(fid, 0);
            C.row(vind) << 1, 0, 0;
            viewer.data().set_colors(C);
            return true;
        }
        cout << "Not in mesh!" << endl;
        return false;
    }

    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods)
    {
        switch (key) {
        case 'n':
            project_sss();
            i += 1;
            return true;
        case 'm':
            project_sss();
            i += 100;
            return true;
        default:
            return false;
        }
    }
};

Eigen::MatrixXd bathy_map_mesh::overlay_sss(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                            const BoundsT& bounds, const xtf_sss_ping::PingsT& pings)
{
    Eigen::MatrixXd C_jet;
    igl::jet(V.col(2), true, C_jet);

    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Nb;
    igl::readSTL("5TUM.stl", Vb, Fb, Nb);
    Eigen::MatrixXd Cb(Vb.rows(), 3);
    Cb.rowwise() = Eigen::RowVector3d(1., 1., 0.);
    Vb.array() *= 0.01;
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()).matrix();
    Vb *= Rz.transpose();
    //display_mesh(Vb, Fb);

    Eigen::Vector3d offset(bounds(0, 0), bounds(0, 1), 0.);
    survey_viewer viewer(V, F, C_jet, Vb, Fb, Cb, pings, offset);
    viewer.launch();

    return C_jet;
}
