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

BathyMapImage::BathyMapImage(mbes_ping::PingsT& pings, int rows, int cols) : rows(rows), cols(cols)
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

void BathyMapImage::draw_track(mbes_ping::PingsT& pings)
{
    draw_track(pings, cv::Scalar(0, 0, 255));
}

void BathyMapImage::draw_track(mbes_ping::PingsT& pings, const cv::Scalar& color)
{
    //nbr_tracks_drawn += 1; // we should based the color on this instead

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    vector<cv::Point2f> curve_points;
    int counter = 0;
    for (const mbes_ping& ping : pings) {
        cv::Point2f pt(x0+res*(ping.pos_[0]-minx), bathy_map.rows-y0-res*(ping.pos_[1]-miny)-1);
        curve_points.push_back(pt);
        //cout << pt << endl;
        if (false) { //counter % 500 == 0) {
            double len = 30.;
            cv::Point pt1(x0+res*(ping.pos_[0]-minx), bathy_map.rows-y0-res*(ping.pos_[1]-miny)-1);
            cv::Point pt2(pt1.x + int(len*cos(ping.heading_)), pt1.y - int(len*sin(ping.heading_)));
            cv::arrowedLine(bathy_map, pt1, pt2, cv::Scalar(0, 0 , 255), 2, 8, 0, 0.1);
            cv::putText(bathy_map, std::to_string(int(180./M_PI*ping.heading_)), pt1, cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(0, 0, 0), 1, 8, false);
        }
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

void BathyMapImage::draw_height_map(mbes_ping::PingsT& pings)
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
