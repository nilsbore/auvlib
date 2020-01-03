/* Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <bathy_maps/mesh_map.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/gl.h>
#include <igl/xml/writeDAE.h>
#include <igl/readPLY.h>
#include <igl/slice.h>
#include <igl/slice_mask.h>
#include <igl/cumsum.h>
#include <igl/ray_mesh_intersect.h>

//#include <igl/copyleft/cgal/intersect_with_half_space.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

using namespace std;

namespace mesh_map {

using namespace std_data;

pair<Eigen::MatrixXd, Eigen::MatrixXi> cut_square_around_point(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                                                               const Eigen::Vector2d& p, double side)
{
    //Eigen::MatrixXd J;
    //igl::copyleft::cgal::intersect_with_half_space(vertices, faces, point, direction, vertices, faces, J);
    Eigen::Array<bool, Eigen::Dynamic, 1> good_V = (V.col(0).array() - p(0)).abs() < 0.5*side && (V.col(1).array() - p(1)).abs() < 0.5*side;

    cout << "Total number vertices: " << V.rows() << endl;

    int nbr_good_V = good_V.cast<int>().sum();
    cout << "Number good vertices: " << nbr_good_V << endl;

    Eigen::VectorXi cum_V;
    Eigen::VectorXi good_V_int = good_V.cast<int>();
    igl::cumsum(good_V_int, 1, cum_V);
    cum_V.array() -= 1;

    cout << "Cout cum_V last row: " << cum_V(cum_V.rows() - 1) << endl;

    Eigen::Array<bool, Eigen::Dynamic, 1> good1, good2, good3, good_F;
    good1 = igl::slice(good_V, F.col(0));
    good2 = igl::slice(good_V, F.col(1));
    good3 = igl::slice(good_V, F.col(2));
    good_F = good1 && good2 && good3;

    int nbr_good_F = good_F.cast<int>().sum();
    cout << "Number good faces: " << nbr_good_F << endl;

    Eigen::MatrixXd new_V = igl::slice_mask(V, good_V, 1);
    cout << "Sliced good vertices: " << new_V.rows() << endl;
    Eigen::MatrixXi small_F = igl::slice_mask(F, good_F, 1);

    Eigen::MatrixXi new_F(small_F.rows(), small_F.cols());
    new_F.col(0) = igl::slice(cum_V, small_F.col(0));
    new_F.col(1) = igl::slice(cum_V, small_F.col(1));
    new_F.col(2) = igl::slice(cum_V, small_F.col(2));

    return make_pair(new_V, new_F);
}

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

void write_dae_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const boost::filesystem::path& filename)
{
    igl::xml::writeDAE(filename.string(), V, F);
}

void write_dae_mesh_from_str(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const string& filename)
{
    write_dae_mesh(V, F, boost::filesystem::path(filename));
}

pair<Eigen::MatrixXd, Eigen::MatrixXi> read_ply_mesh_from_str(const string& filename)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readPLY(filename, V, F);
    return make_pair(V, F);
}

void show_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
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
    viewer.data().show_lines = false;

    //viewer.callback_pre_draw = std::bind(&IglVisCallback::callback_pre_draw, this, std::placeholders::_1);
    //viewer.callback_key_pressed = std::bind(&IglVisCallback::callback_key_pressed, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    viewer.core().is_animating = true;
    viewer.core().animation_max_fps = 30.;
	//viewer.launch();
    viewer.core().background_color << 1., 1., 1., 1.; // white background
	viewer.launch();
}

std::tuple<Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>,
           Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>,
           Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> >
height_map_to_texture(const Eigen::MatrixXd& height_map)
{
    Eigen::ArrayXXd height_map_array = height_map.array();
    double minv = height_map.minCoeff();
    height_map_array -= minv*(height_map_array < 0).cast<double>();
    double maxv = height_map_array.maxCoeff();
    height_map_array /= maxv;

    int rows = height_map.rows();
    int cols = height_map.cols();

    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> R(cols, rows);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> G(cols, rows);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> B(cols, rows);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            tie(R(j, i), G(j, i), B(j, i)) = jet_mesh(height_map_array(i, j));
        }
    }

    return make_tuple(R, G, B);
}

//void show_textured_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& height_map, const BoundsT& bounds)
void show_textured_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                        const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
                        const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
                        const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B,
                        const BoundsT& bounds)
{
    /*
    Eigen::ArrayXXd height_map_array = height_map.array();
    double minv = height_map.minCoeff();
    height_map_array -= minv*(height_map_array < 0).cast<double>();
    double maxv = height_map_array.maxCoeff();
    height_map_array /= maxv;

    int rows = height_map.rows();
    int cols = height_map.cols();

    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> R(cols, rows);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> G(cols, rows);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> B(cols, rows);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            tie(R(j, i), G(j, i), B(j, i)) = jet_mesh(height_map_array(i, j));
        }
    }
    */

    /*
    Eigen::MatrixXd UV(4,2);
    UV << bounds(0, 0), bounds(0, 1),
          bounds(1, 0), bounds(0, 1),
          bounds(1, 0), bounds(1, 1),
          bounds(0, 0), bounds(1, 1);
    */
    Eigen::MatrixXd UV = V.leftCols<2>();
    //UV.col(0).array() -= bounds(0, 0);
    UV.col(0).array() /= bounds(1, 0) - bounds(0, 0);
    //UV.col(1).array() -= bounds(0, 1);
    UV.col(1).array() /= bounds(1, 1) - bounds(0, 1);

    igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);
    viewer.data().set_uv(UV);
    // Add per-vertex colors
    //viewer.data().set_colors(C);

    //Eigen::MatrixXd C_jet;
    //igl::jet(V.col(2), true, C_jet);
    // Add per-vertex colors
    //viewer.data().set_colors(C_jet);

    viewer.data().show_texture = true;

    // Use the image as a texture
    viewer.data().set_texture(R,G,B);

    viewer.data().point_size = 10;
    viewer.data().line_width = 1;
    viewer.data().show_lines = false;

    //viewer.callback_pre_draw = std::bind(&IglVisCallback::callback_pre_draw, this, std::placeholders::_1);
    //viewer.callback_key_pressed = std::bind(&IglVisCallback::callback_key_pressed, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    viewer.core().is_animating = true;
    viewer.core().animation_max_fps = 30.;
	//viewer.launch();
    viewer.core().background_color << 1., 1., 1., 1.; // white background
	viewer.launch();
}

void show_height_map(const Eigen::MatrixXd& height_map)
{
    double minv = height_map.minCoeff();
    Eigen::ArrayXXd height_map_array = height_map.array();
    height_map_array -= minv*(height_map_array < 0).cast<double>();
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

pair<Eigen::MatrixXd, BoundsT> height_map_from_pings(const mbes_ping::PingsT& pings, double res)
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

pair<Eigen::MatrixXd, BoundsT> height_map_from_cloud(const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& cloud, double res)
{
    auto xcomp = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return p1[0] < p2[0];
    };
    auto ycomp = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return p1[1] < p2[1];
    };
    double maxx = std::max_element(cloud.begin(), cloud.end(), xcomp)->x();
    double minx = std::min_element(cloud.begin(), cloud.end(), xcomp)->x();
    double maxy = std::max_element(cloud.begin(), cloud.end(), ycomp)->y();
    double miny = std::min_element(cloud.begin(), cloud.end(), ycomp)->y();

    cout << "Min X: " << minx << ", Max X: " << maxx << ", Min Y: " << miny << ", Max Y: " << maxy << endl;

    int cols = std::ceil((maxx-minx)/res);
    int rows = std::ceil((maxy-miny)/res);
    maxx = minx + double(cols)*res;
    maxy = miny + double(rows)*res;

    cout << "Target res: " << res << endl;
    cout << "Initial cols: " << cols << ", rows: " << rows << endl;

    Eigen::MatrixXd means(rows, cols); means.setZero();
    Eigen::MatrixXd counts(rows, cols); counts.setZero();

    for (const Eigen::Vector3d& pos : cloud) {
        int col = int((pos[0]-minx)/res);
        int row = int((pos[1]-miny)/res);
        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            means(row, col) += pos[2];
            counts(row, col) += 1.;
        }
    }

    Eigen::ArrayXXd counts_pos = counts.array() + (counts.array() == 0.).cast<double>();
    means.array() /= counts_pos;

    BoundsT bounds; bounds << minx, miny, maxx, maxy;

    return make_pair(means, bounds);
}

pair<Eigen::MatrixXd, BoundsT> height_map_from_dtm_cloud(const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& cloud, double res)
{
    auto xcomp = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return p1[0] < p2[0];
    };
    auto ycomp = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return p1[1] < p2[1];
    };
    double maxx = std::max_element(cloud.begin(), cloud.end(), xcomp)->x();
    double minx = std::min_element(cloud.begin(), cloud.end(), xcomp)->x();
    double maxy = std::max_element(cloud.begin(), cloud.end(), ycomp)->y();
    double miny = std::min_element(cloud.begin(), cloud.end(), ycomp)->y();

    BoundsT bounds; bounds << minx, miny, maxx, maxy;

    double float_cols = (maxx - minx) / res; // min/max should be in the center
    double float_rows = (maxy - miny) / res;

    cout << "Number of cols: " << float_cols << endl;
    cout << "Number of rows: " << float_rows << endl;

    int rows = int(float_rows);
    int cols = int(float_cols);

    Eigen::MatrixXd height_map(rows, cols); height_map.setZero();
    for (const Eigen::Vector3d& pos : cloud) {
        int col = int((pos[0]-minx)/res);
        int row = int((pos[1]-miny)/res);
        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            height_map(row, col) = pos[2];
        }
    }

    return make_pair(height_map, bounds);
}

pair<Eigen::MatrixXd, Eigen::MatrixXi> mesh_from_height_map(const Eigen::MatrixXd& height_map, const BoundsT& bounds)
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

tuple<Eigen::MatrixXd, Eigen::MatrixXi, BoundsT> mesh_from_pings(const mbes_ping::PingsT& pings, double res)
{
    Eigen::MatrixXd height_map;
    BoundsT bounds;
    tie(height_map, bounds) = height_map_from_pings(pings, res);
    //display_height_map(height_map);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    tie(V, F) = mesh_from_height_map(height_map, bounds);
    return make_tuple(V, F, bounds);
}

tuple<Eigen::MatrixXd, Eigen::MatrixXi, BoundsT> mesh_from_cloud(const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& cloud, double res)
{
    Eigen::MatrixXd height_map;
    BoundsT bounds;
    tie(height_map, bounds) = height_map_from_cloud(cloud, res);
    //display_height_map(height_map);
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    tie(V, F) = mesh_from_height_map(height_map, bounds);
    return make_tuple(V, F, bounds);
}

// In this case, res gives us the spacing between the grid points
tuple<Eigen::MatrixXd, Eigen::MatrixXi, BoundsT> mesh_from_dtm_cloud(const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& cloud, double res)
{
    Eigen::MatrixXd height_map;
    BoundsT bounds;
    tie(height_map, bounds) = height_map_from_dtm_cloud(cloud, res);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    tie(V, F) = mesh_from_height_map(height_map, bounds);

    return make_tuple(V, F, bounds);
}

double depth_at_point(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::Vector3d& origin)
{
    igl::Hit hit;
    bool did_hit = ray_mesh_intersect(origin, Eigen::Vector3d(0., 0., -1.), V, F, hit);
    return did_hit? origin(2) - V(F(hit.id, 0), 2) : 0.;
}

Eigen::MatrixXd compute_normals(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    Eigen::MatrixXd N;
    igl::per_vertex_normals(V, F, N);
    return N;
}

tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd, BoundsT> mesh_and_normals_from_pings(const mbes_ping::PingsT& pings, double res)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    BoundsT bounds;
    tie(V, F, bounds) = mesh_from_pings(pings, res);
    cout << bounds << endl;

    Eigen::MatrixXd N = compute_normals(V, F);
    
    return make_tuple(V, F, N, bounds);
}

Eigen::MatrixXd shade_image_from_normals(const Eigen::MatrixXd& N, const BoundsT& bounds, double res, const Eigen::Vector3d& light_dir)
{
    int cols = std::ceil((bounds(1, 0) - bounds(0, 0)) / res); // min/max should be in the center
    int rows = std::ceil((bounds(1, 1) - bounds(0, 1)) / res);

    cout << "Rows: " << rows << ", Cols: " << cols << endl;

    Eigen::Vector3d norm_light_dir = 1./light_dir.norm()*light_dir;

    Eigen::MatrixXd shade_image(rows, cols); shade_image.setZero();
    for (int y = 0; y < rows; ++y) { // ROOM FOR SPEEDUP
	    for (int x = 0; x < cols; ++x) {
            int ind = y*cols+x;
            if (!std::isnan(N(ind, 0))) {
                shade_image(y, x) = fabs(norm_light_dir.dot(N.row(ind)));
            }
        }
    }

    return shade_image;
}

Eigen::MatrixXd normals_at_points(const Eigen::MatrixXd& points, const Eigen::MatrixXd& N, const BoundsT& bounds, double res)
{
    int cols = std::ceil((bounds(1, 0) - bounds(0, 0)) / res); // min/max should be in the center
    int rows = std::ceil((bounds(1, 1) - bounds(0, 1)) / res);

    cout << "Rows: " << rows << ", Cols: " << cols << endl;

    Eigen::MatrixXd N_points = Eigen::MatrixXd::Zero(points.rows(), points.cols());

    for (int i = 0; i < points.rows(); ++i) {
        //int x = int((points(i, 0)-bounds(0, 0))/res);
        //int y = int((points(i, 1)-bounds(0, 1))/res);
        int x = int(points(i, 0)/res);
        int y = int(points(i, 1)/res);
        N_points.row(i) = N.row(y*cols+x);
    }

    return N_points;
}

} // namespace mesh_map
