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

#include <bathy_maps/draw_map.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace std_data;

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

// OpenCV 2.4.9 which is the default on Ubuntu 18.04 and earlier versions
// do not have the arrowedLine function
void draw_arrowed_line(cv::Mat& img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color,
                       int thickness, int line_type, int shift, double tipLength)
{
    const double tipSize = norm(pt1-pt2)*tipLength; // Factor to normalize the size of the tip depending on the length of the arrow

    cv::line(img, pt1, pt2, color, thickness, line_type, shift);

    const double angle = atan2( (double) pt1.y - pt2.y, (double) pt1.x - pt2.x );

    cv::Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
    cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
    cv::line(img, p, pt2, color, thickness, line_type, shift);

    p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
    p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
    cv::line(img, p, pt2, color, thickness, line_type, shift);
}

BathyMapImage::BathyMapImage(const mbes_ping::PingsT& pings, int rows, int cols) : rows(rows), cols(cols)
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

    double xres = double(cols)/(maxx - minx);
    double yres = double(rows)/(maxy - miny);

    double res = std::min(xres, yres);

    double x0 = .5*(double(cols) - res*(maxx-minx));
    double y0 = .5*(double(rows) - res*(maxy-miny));

    cout << xres << ", " << yres << endl;

    params = array<double, 5>{res, minx, miny, x0, y0};
    bathy_map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
}

BathyMapImage::BathyMapImage(const Eigen::MatrixXd& height_map, const Eigen::Matrix2d& bounds) : rows(height_map.rows()), cols(height_map.cols())
{
    double maxx = bounds(1, 0);
    double minx = bounds(0, 0);
    double maxy = bounds(1, 1);
    double miny = bounds(0, 1);

    cout << "Min X: " << minx << ", Max X: " << maxx << ", Min Y: " << miny << ", Max Y: " << maxy << endl;

    double xres = double(cols)/(maxx - minx);
    double yres = double(rows)/(maxy - miny);

    double res = std::min(xres, yres);

    double x0 = .5*(double(cols) - res*(maxx-minx));
    double y0 = .5*(double(rows) - res*(maxy-miny));

    cout << xres << ", " << yres << endl;

    params = array<double, 5>{res, minx, miny, x0, y0};
    bathy_map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
}

void BathyMapImage::draw_track(const mbes_ping::PingsT& pings)
{
    draw_track(pings, cv::Scalar(0, 0, 255));
}

void BathyMapImage::draw_track(const mbes_ping::PingsT& pings, const cv::Scalar& color)
{
    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    vector<cv::Point2f> curve_points;
    int counter = 0;
    for (const mbes_ping& ping : pings) {
        cv::Point2f pt(x0+res*(ping.pos_[0]-minx), bathy_map.rows-y0-res*(ping.pos_[1]-miny)-1);
        curve_points.push_back(pt);
        ++counter;
    }

    cv::Mat curve(curve_points, true);
    curve.convertTo(curve, CV_32S); //adapt type for polylines
    cv::polylines(bathy_map, curve, false, color, 1, CV_AA);
}

void BathyMapImage::draw_track(const vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& pos)
{
    cv::Scalar color(0, 0, 255);

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    vector<cv::Point2f> curve_points;
    int counter = 0;
    for (const Eigen::Vector3d& p : pos) {
        cv::Point2f pt(x0+res*p[0], bathy_map.rows-y0-res*p[1]-1);
        curve_points.push_back(pt);
        ++counter;
    }

    cv::Mat curve(curve_points, true);
    curve.convertTo(curve, CV_32S); //adapt type for polylines
    cv::polylines(bathy_map, curve, false, color, 1, CV_AA);
}

void BathyMapImage::draw_indices(mbes_ping::PingsT& pings, int skip_indices)
{
    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    int counter = 0;
    for (const mbes_ping& ping : pings) {
        if (counter % skip_indices == 0) {
            double len = 30.;
            cv::Point pt1(x0+res*(ping.pos_[0]-minx), bathy_map.rows-y0-res*(ping.pos_[1]-miny)-1);
            cv::Point pt2(pt1.x + int(len*cos(ping.heading_)), pt1.y - int(len*sin(ping.heading_)));
            //cv::arrowedLine(bathy_map, pt1, pt2, cv::Scalar(0, 0 , 255), 2, 8, 0, 0.1);
            cv::putText(bathy_map, std::to_string(counter), pt1, cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(0, 0, 0), 1, 8, false);
        }
        ++counter;
    }
}

void BathyMapImage::draw_pose(const Eigen::Vector3d& pos, double heading, const cv::Scalar& color)
{
    double len = 30.;
    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    cv::Point pt1(x0+res*(pos[0]-minx), bathy_map.rows-y0-res*(pos[1]-miny)-1);
    cv::Point pt2(pt1.x + int(len*cos(heading)), pt1.y - int(len*sin(heading)));
    draw_arrowed_line(bathy_map, pt1, pt2, color, 2, 8, 0, 0.1);
}

void BathyMapImage::draw_red_pose(const Eigen::Vector3d& pos, double heading)
{
    draw_pose(pos, heading, cv::Scalar(0, 0, 255));
}

void BathyMapImage::draw_blue_pose(const Eigen::Vector3d& pos, double heading)
{
    draw_pose(pos, heading, cv::Scalar(255, 0, 0));
}

void BathyMapImage::draw_height_map(const Eigen::MatrixXd& height_map)
{
    double minv = height_map.minCoeff();
    Eigen::ArrayXXd height_map_array = height_map.array();
    height_map_array -= minv*(height_map_array < 0).cast<double>();
    double maxv = height_map_array.maxCoeff();
    height_map_array /= maxv;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (height_map_array(i, j) == 0) {
                continue;
            }
            cv::Point3_<uchar>* p = bathy_map.ptr<cv::Point3_<uchar> >(rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(height_map_array(i, j));
        }
    }

    cout << "Min value: " << minv << ", max value: " << minv+maxv << endl;
}

void BathyMapImage::draw_height_map(const mbes_ping::PingsT& pings)
{
    Eigen::MatrixXd means(rows, cols); means.setZero();
    Eigen::MatrixXd counts(rows, cols); counts.setZero();

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    for (const mbes_ping& ping : pings) {
        for (const Eigen::Vector3d& pos : ping.beams) {
            int col = int(x0+res*(pos[0]-minx));
            int row = int(y0+res*(pos[1]-miny));
            if (col >= 0 && col < cols && row >= 0 && row < rows) {
                means(row, col) += pos[2];
                counts(row, col) += 1.;
            }
        }
    }

    Eigen::ArrayXXd counts_pos = counts.array() + (counts.array() == 0.).cast<double>();
    means.array() /= counts_pos;

    double minv = means.minCoeff();
    double meanv = means.mean();

    means.array() -= minv*(counts.array() > 0).cast<double>();
    double maxv = means.maxCoeff();
    means.array() /= maxv;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (means(i, j) == 0) {
                continue;
            }
            cv::Point3_<uchar>* p = bathy_map.ptr<cv::Point3_<uchar> >(rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(means(i, j));
        }
    }

    cout << "Min value: " << minv << ", max value: " << minv+maxv << endl;
}

void BathyMapImage::draw_back_scatter_map(mbes_ping::PingsT& pings)
{
    mbes_ping::PingsT back_scatter_pings = pings;
    for (mbes_ping& ping : back_scatter_pings) {
        for (int i = 0; i < ping.beams.size(); ++i) {
            //ping.beams[i](2) = std::max(std::min(200., ping.back_scatter[i]), 50.);
            //ping.beams[i](2) = std::min(ping.back_scatter[i]*ping.beams[i].norm()/20., 500.);
            double dist = (ping.beams[i] - ping.pos_).norm();
            //ping.beams[i](2) = std::min(std::max(ping.back_scatter[i]*ping.beams[i].norm()/20., 50.), 1000.);
            ping.beams[i](2) = std::max(std::min(ping.back_scatter[i]*dist/20., 300.), 50.);
        }
    }
    draw_height_map(back_scatter_pings);
}

void BathyMapImage::draw_targets(const TargetsT& targets, const cv::Scalar& color)
{
    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    for (const pair<string, pair<double, double> >& item : targets) {
        int col = int(x0+res*(item.second.first-minx));
        int row = rows-int(y0+res*(item.second.second-miny))-1;
        cv::circle(bathy_map, cv::Point(col, row), 10, color); //, int thickness=1, int lineType=8, int shift=0)
        cv::putText(bathy_map, item.first, cv::Point(col+15, row), cv::FONT_HERSHEY_PLAIN, 1.0, color); //, 1, 8, false);
    }

}

cv::Point2f BathyMapImage::world_pos_to_image(const Eigen::Vector3d& pos, bool relative)
{
    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];
    if (relative) {
        minx = miny = 0.;
    }
    return cv::Point(x0+res*(pos[0]-minx), bathy_map.rows-y0-res*(pos[1]-miny)-1);
}

void BathyMapImage::rotate_crop_image(const Eigen::Vector3d& first_pos, const Eigen::Vector3d& last_pos, double result_width)
{
    double res = params[0];
    double margin = 20.;

    Eigen::Vector3d center_pos = .5*(last_pos + first_pos);
    Eigen::Vector3d forward_dir = last_pos - first_pos;
    double dist = forward_dir.norm();
    forward_dir.normalize();
    double forward_angle = 180./M_PI*atan2(forward_dir(0), forward_dir(1));

    cv::Size2f rect_size(result_width*res, (dist + margin)*res);
    cv::RotatedRect rect = cv::RotatedRect(world_pos_to_image(center_pos, true), rect_size, forward_angle);

    cv::Mat M, rotated; //, cropped;
    float angle = rect.angle;
    /*
    if (rect.angle < -45.) {
        angle += 90.0;
        swap(rect_size.width, rect_size.height);
    }
    */
    cv::Point2f center((bathy_map.cols-1)/2.0, (bathy_map.rows-1)/2.0);
    M = cv::getRotationMatrix2D(center, angle, 1.0);
    //cv::Size margin_size(2*bathy_map.cols, 2*bathy_map.rows);
    cv::Rect bbox = cv::RotatedRect(cv::Point2f(), bathy_map.size(), angle).boundingRect();
    // adjust transformation matrix
    M.at<double>(0, 2) += bbox.width/2.0 - bathy_map.cols/2.0;
    M.at<double>(1, 2) += bbox.height/2.0 - bathy_map.rows/2.0;
        //
    cv::warpAffine(bathy_map, rotated, M, bbox.size(), cv::INTER_CUBIC);
    cv::Mat_<double> src(3/*rows*/,1 /* cols */); 
    src(0, 0) = rect.center.x; 
    src(1, 0) = rect.center.y;
    src(2, 0) = 1.;

    cv::Mat_<double> dst = M*src; //USE MATRIX ALGEBRA 
    cv::getRectSubPix(rotated, rect_size, cv::Point2f(dst(0,0),dst(1,0)), bathy_map);
    //cv::imshow("Cropped", cropped);
    //cv::waitKey();
}

void BathyMapImage::write_image(const boost::filesystem::path& path)
{
    cv::imwrite(path.string(), bathy_map);
}

void BathyMapImage::write_image_from_str(const std::string& path)
{
    write_image(boost::filesystem::path(path));
}

void BathyMapImage::show()
{
    cv::imshow("Bathy image", bathy_map);
    cv::waitKey();
}

void BathyMapImage::blip()
{
    cv::imshow("Bathy image", bathy_map);
    cv::waitKey(1);
}
