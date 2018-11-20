#include <bathy_maps/mesh_map.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>

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


