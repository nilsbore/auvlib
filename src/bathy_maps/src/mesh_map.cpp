#include <bathy_maps/mesh_map.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>
#include <igl/readSTL.h>
#include <igl/ray_mesh_intersect.h>
#include <igl/embree/line_mesh_intersection.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

std::tuple<uint8_t, uint8_t, uint8_t> jet(double x)
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
            tie(p->z, p->y, p->x) = jet(height_map_array(i, j));
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

tuple<Eigen::MatrixXd, Eigen::MatrixXi, bathy_map_mesh::BoundsT> bathy_map_mesh::mesh_from_pings(const mbes_ping::PingsT& pings)
{
    Eigen::MatrixXd height_map;
    BoundsT bounds;
    tie(height_map, bounds) = height_map_from_pings(pings, 0.5);
    //display_height_map(height_map);
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

    survey_viewer(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
        const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
        const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset)
        : pings(pings), i(0), V1(V1), F1(F1), V2(V2), F2(F2), offset(offset)
    {
        //double first_heading = pings[0].heading_;

        V = Eigen::MatrixXd(V1.rows() + V2.rows(), V1.cols());
        V << V1, this->V2;
        F = Eigen::MatrixXi(F1.rows() + F2.rows(), F1.cols());
        F << F1, (F2.array() + V1.rows());

        V.bottomRows(V2.rows()) = V2;
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[0].heading_, Eigen::Vector3d::UnitZ()).matrix();
        V.bottomRows(V2.rows()) *= Rz.transpose();
        V.bottomRows(V2.rows()).array().rowwise() += (pings[0].pos_ - offset).transpose().array();

        Eigen::MatrixXd C = Eigen::MatrixXd(C1.rows()+C2.rows(), C1.cols());
        C << C1, C2;

        viewer.data().set_mesh(V, F);
        // Add per-vertex colors
        //viewer.data().set_colors(C);
        // Add per-vertex colors
        viewer.data().set_colors(C);

        viewer.data().point_size = 10;
        viewer.data().line_width = 1;

        //viewer.callback_pre_draw = std::bind(&IglVisCallback::callback_pre_draw, this, std::placeholders::_1);
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
        const int nbr_lines = 40;

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

    pair<Eigen::MatrixXd, Eigen::MatrixXd> compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width)
    {
        igl::Hit hit;
        Eigen::MatrixXd dirs_left;
        Eigen::MatrixXd dirs_right;
        tie(dirs_left, dirs_right) = compute_sss_dirs(R, tilt_angle, beam_width);
        Eigen::MatrixXd hits_left(dirs_left.rows(), 3);
        Eigen::MatrixXd hits_right(dirs_right.rows(), 3);
        int hit_count = 0;
        for (int i = 0; i < dirs_left.rows(); ++i) {
            bool did_hit = ray_mesh_intersect(origin, dirs_left.row(i).transpose(), V1, F1, hit);
            if (did_hit) {
                int vind = F1(hit.id, 0);
                // we actually get the coordinates within the triangle also, we could use that
                hits_left.row(hit_count) = V1.row(vind);
                ++hit_count;
            }
        }
        hits_left.conservativeResize(hit_count, 3);
        hit_count = 0;
        for (int i = 0; i < dirs_right.rows(); ++i) {
            bool did_hit = ray_mesh_intersect(origin, dirs_right.row(i).transpose(), V1, F1, hit);
            if (did_hit) {
                int vind = F1(hit.id, 0);
                // we actually get the coordinates within the triangle also, we could use that
                hits_right.row(hit_count) = V1.row(vind);
                ++hit_count;
            }
        }
        hits_right.conservativeResize(hit_count, 3);
        return make_pair(hits_left, hits_right);
    }

    pair<Eigen::MatrixXd, Eigen::MatrixXd> embree_compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width)
    {
        igl::Hit hit;
        Eigen::MatrixXd dirs_left;
        Eigen::MatrixXd dirs_right;
        tie(dirs_left, dirs_right) = compute_sss_dirs(R, tilt_angle, beam_width);
        Eigen::MatrixXd hits_left(dirs_left.rows(), 3);
        Eigen::MatrixXd hits_right(dirs_right.rows(), 3);

        int nbr_lines = dirs_left.rows() + dirs_right.rows();
        Eigen::MatrixXd dirs = Eigen::MatrixXd(nbr_lines, dirs_left.cols());
        dirs << dirs_left, dirs_right;
        Eigen::MatrixXd P = Eigen::MatrixXd(nbr_lines, 3);
        P.rowwise() = origin.transpose();
        Eigen::MatrixXd hits = igl::embree::line_mesh_intersection(P, dirs, V1, F1);

        int hit_count = 0;
        for (int i = 0; i < dirs_left.rows(); ++i) {
            int hit = hits(i, 0);
            if (hit > 0) {
                int vind = F1(hit, 0);
                // we actually get the coordinates within the triangle also, we could use that
                hits_left.row(hit_count) = V1.row(vind);
                ++hit_count;
            }
        }
        hits_left.conservativeResize(hit_count, 3);
        hit_count = 0;
        for (int i = 0; i < dirs_right.rows(); ++i) {
            int hit = hits(dirs_left.rows() + i, 0);
            if (hit > 0) {
                int vind = F1(hit, 0);
                // we actually get the coordinates within the triangle also, we could use that
                hits_right.row(hit_count) = V1.row(vind);
                ++hit_count;
            }
        }
        hits_right.conservativeResize(hit_count, 3);
        return make_pair(hits_left, hits_right);
    }

    bool callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods)
    {
        switch (key) {
        case 'n':
            if (i < pings.size()) {
                cout << "Setting new position: " << pings[i].pos_.transpose() << endl;
                V.bottomRows(V2.rows()) = V2;
                Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
                V.bottomRows(V2.rows()) *= Rz.transpose();
                V.bottomRows(V2.rows()).array().rowwise() += (pings[i].pos_ - offset).transpose().array();
                viewer.data().set_vertices(V);
                //viewer.data().compute_normals();

                Eigen::MatrixXd hits_left;
                Eigen::MatrixXd hits_right;
                //tie(hits_left, hits_right) = compute_hits(pings[i].pos_ - offset, Rz, pings[i].port.tilt_angle, pings[i].port.beam_width);
                tie(hits_left, hits_right) = embree_compute_hits(pings[i].pos_ - offset, Rz, pings[i].port.tilt_angle, pings[i].port.beam_width);
                Eigen::MatrixXi E;
                Eigen::MatrixXd P(hits_left.rows(), 3);
                P.rowwise() = (pings[i].pos_ - offset).transpose();
                viewer.data().set_edges(P, E, Eigen::RowVector3d(1., 0., 0.));
                viewer.data().add_edges(P, hits_left, Eigen::RowVector3d(1., 0., 0.));
                P = Eigen::MatrixXd(hits_right.rows(), 3);
                P.rowwise() = (pings[i].pos_ - offset).transpose();
                viewer.data().add_edges(P, hits_right, Eigen::RowVector3d(0., 1., 0.));
                i += 10;
            }
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
