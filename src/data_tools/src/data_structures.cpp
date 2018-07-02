#include <data_tools/data_structures.h>
#include <data_tools/colormap.h>

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

// instantiate all versions needed to read the structs
template nav_entry::EntriesT read_data<nav_entry::EntriesT>(const boost::filesystem::path& path);
template mbes_ping::PingsT read_data<mbes_ping::PingsT>(const boost::filesystem::path& path);
template pt_submaps read_data<pt_submaps>(const boost::filesystem::path& path);
template gp_submaps read_data<gp_submaps>(const boost::filesystem::path& path);
template track_error_benchmark read_data<track_error_benchmark>(const boost::filesystem::path& path);

// instantiate all versions needed to write the structs
template void write_data<nav_entry::EntriesT>(nav_entry::EntriesT& data, const boost::filesystem::path& path);
template void write_data<mbes_ping::PingsT>(mbes_ping::PingsT& data, const boost::filesystem::path& path);
template void write_data<pt_submaps>(pt_submaps& data, const boost::filesystem::path& path);
template void write_data<gp_submaps>(gp_submaps& data, const boost::filesystem::path& path);
template void write_data<track_error_benchmark>(track_error_benchmark& data, const boost::filesystem::path& path);

// res, xmin, ymin, imxmin, imymin
void track_error_benchmark::track_img_params(mbes_ping::PingsT& pings, int rows, int cols)
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
    track_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
}

void track_error_benchmark::draw_track_img(mbes_ping::PingsT& pings, cv::Mat& img, const cv::Scalar& color, const std::string& name)
{
    //nbr_tracks_drawn += 1; // we should based the color on this instead

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    vector<cv::Point2f> curve_points;
    int counter = 0;
    for (const mbes_ping& ping : pings) {
        cv::Point2f pt(x0+res*(ping.pos_[0]-minx), img.rows-y0-res*(ping.pos_[1]-miny)-1);
        curve_points.push_back(pt);
        //cout << pt << endl;
        if (counter % 500 == 0) {
            double len = 30.;
            cv::Point pt1(x0+res*(ping.pos_[0]-minx), img.rows-y0-res*(ping.pos_[1]-miny)-1);
            cv::Point pt2(pt1.x + int(len*cos(ping.heading_)), pt1.y - int(len*sin(ping.heading_)));
            cv::arrowedLine(img, pt1, pt2, cv::Scalar(0, 0 , 255), 2, 8, 0, 0.1);
            cv::putText(img, std::to_string(int(180./M_PI*ping.heading_)), pt1, cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(0, 0, 0), 1, 8, false);
        }
        ++counter;
    }

    cv::Mat curve(curve_points, true);
    curve.convertTo(curve, CV_32S); //adapt type for polylines
    cv::polylines(img, curve, false, color, 1, CV_AA);
    
    //track_img_path = "track.png";
}

/*
void track_error_benchmark::draw_track_img(pt_submaps::TransT& positions)
{
    nbr_tracks_drawn += 1; // we should based the color on this instead

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    vector<cv::Point2f> curve_points;
    for (const Eigen::Vector3d pos : positions) {
        cv::Point2f pt(x0+res*(pos[0]-minx), y0+res*(pos[1]-miny));
        curve_points.push_back(pt);
        //cout << pt << endl;
    }

    cv::Mat curve(curve_points, true);
    curve.convertTo(curve, CV_32S); //adapt type for polylines
    cv::polylines(track_img, curve, false, cv::Scalar(colormap[nbr_tracks_drawn][2], colormap[nbr_tracks_drawn][1], colormap[nbr_tracks_drawn][0]), 1, CV_AA);
    
    track_img_path = "track.png";
}
*/

void track_error_benchmark::draw_track_legend()
{
    /*
    index = 0;
    for (const auto& s : series_) {
        if (!s.legend()) {
            continue;
        }
        auto name = s.label();
        int baseline;
        cv::Size size = getTextSize(name, cv::FONT_HERSHEY_SIMPLEX, 0.4f, 1.f, &baseline);
        cv::Point org(buffer.cols - border_size_ - size.width - 17,
            border_size_ + 15 * index + 15);
        auto shadow = true;
        cv::putText(trans.with(background_color_), name.c_str(),
            { org.x + (shadow ? 1 : 0), org.y + (shadow ? 1 : 0) },
            cv::FONT_HERSHEY_SIMPLEX, 0.4f, color2scalar(background_color_),
            (shadow ? 1.f : 2.f));
        cv::circle(trans.with(background_color_),
            { buffer.cols - border_size_ - 10 + 1, org.y - 3 + 1 }, 3,
            color2scalar(background_color_), -1, CV_AA);
        cv::putText(trans.with(text_color_), name.c_str(), org,
            cv::FONT_HERSHEY_SIMPLEX, 0.4f, color2scalar(text_color_), 1.f);
        s.dot(&trans.with(s.color()), buffer.cols - border_size_ - 10, org.y - 3,
            3);
        index++;
    }
    */
}

double track_error_benchmark::compute_rms_error(mbes_ping::PingsT& pings)
{
    if (pings.size() != gt_track.size()) {
        cout << "Track length not same as ground truth!" << endl;
        exit(-1);
    }

    double rms_error = 0.; double count = 0.;
    for (int i = 0; i < gt_track.size(); ++i) {
        rms_error += (pings[i].pos_-gt_track[i]).squaredNorm();
        count += 1.;
    }

    return sqrt(rms_error/count);
}

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

pair<double, cv::Mat> track_error_benchmark::compute_draw_consistency_map(mbes_ping::PingsT& pings)
{
    int rows = 500;
    int cols = 500;

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

    /*
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (counts(i, j) == 0) {
                counts(i, j) = 1;
            }
        }
    }
    */

    counts.array() += (counts.array() == 0.).cast<double>();

    means.array() /= counts.array();
    
    Eigen::MatrixXd mean_offsets(rows, cols); mean_offsets.setZero();
    for (const mbes_ping& ping : pings) {
        for (const Eigen::Vector3d& pos : ping.beams) {
            int col = int(x0+res*(pos[0]-minx));
            int row = int(y0+res*(pos[1]-miny));
            if (col >= 0 && col < cols && row >= 0 && row < rows) {
                mean_offsets(row, col) += (pos[2] - means(row, col))*(pos[2] - means(row, col));
            }
        }
    }

    mean_offsets.array() /= counts.array();
    mean_offsets.array() = mean_offsets.array().sqrt();

    double maxv = mean_offsets.maxCoeff();
    double minv = mean_offsets.minCoeff();
    double meanv = mean_offsets.mean();

    mean_offsets.array() -= minv;
    mean_offsets.array() /= (maxv - minv);

    cv::Mat error_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (mean_offsets(i, j) == 0) {
                continue;
            }
            cv::Point3_<uchar>* p = error_img.ptr<cv::Point3_<uchar> >(rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(mean_offsets(i, j));
        }
    }

    //cv::imshow("My image", error_img);
    //cv::waitKey();

    return make_pair(meanv, error_img);
}

cv::Mat track_error_benchmark::draw_height_map(mbes_ping::PingsT& pings)
{
    int rows = 500;
    int cols = 500;

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

    cv::Mat mean_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (means(i, j) == 0) {
                continue;
            }
            cv::Point3_<uchar>* p = mean_img.ptr<cv::Point3_<uchar> >(rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(means(i, j));
        }
    }

    return mean_img;
}

void track_error_benchmark::add_ground_truth(mbes_ping::PingsT& pings)
{
    int rows = 500;
    int cols = 500;
    for (mbes_ping& ping : pings) {
        gt_track.push_back(ping.pos_);
    }
    track_img_params(pings, rows, cols);
    add_benchmark(pings, "ground_truth");
    track_img_path = "benchmark_track_img.png";
}

void track_error_benchmark::add_initial(mbes_ping::PingsT& pings)
{
    input_pings = pings;
}

void track_error_benchmark::add_benchmark(mbes_ping::PingsT& pings, const std::string& name)
{
    int nbr_tracks_drawn = track_rms_errors.size();
    cv::Scalar color(colormap[nbr_tracks_drawn][2], colormap[nbr_tracks_drawn][1], colormap[nbr_tracks_drawn][0]);
    draw_track_img(pings, track_img, color, name);
    cv::Mat error_img;
    double consistency_rms_error;
    tie(consistency_rms_error, error_img) = compute_draw_consistency_map(pings);
    draw_track_img(pings, error_img, cv::Scalar(0, 0, 0), name);
    string error_img_path = name + "_rms_consistency_error.png";
    cv::imwrite(error_img_path, error_img);
    error_img_paths[name] = error_img_path;
    consistency_rms_errors[name] = consistency_rms_error;
    
    cv::Mat mean_img = draw_height_map(pings);
    draw_track_img(pings, mean_img, cv::Scalar(0, 0, 0), name);
    string mean_img_path = name + "_mean_depth.png";
    cv::imwrite(mean_img_path, mean_img);

    double track_rms_error = compute_rms_error(pings);
    track_rms_errors[name] = track_rms_error;

    cout << "Added benchmark " << name << endl;
    cout << "RMS track error: " << track_rms_error << endl;
    cout << "RMS consistency error: " << consistency_rms_error << endl;
    cout << "Consistency image map: " << error_img_path << endl;

}

void track_error_benchmark::add_benchmark(pt_submaps::TransT& trans_corr, pt_submaps::RotsT& rots_corr, const std::string& name)
{
    mbes_ping::PingsT pings = input_pings;

    int i = 0;
    Eigen::Vector3d tc;
    Eigen::Matrix3d Rc;
    for (mbes_ping& ping : pings) {
        if (ping.first_in_file_) {
            tc = trans_corr[i];
            Rc = rots_corr[i];
            i += 1;
        }
        ping.pos_ = Rc*ping.pos_ + tc;
        for (Eigen::Vector3d& p : ping.beams) {
            p = Rc*p + tc;
        }
    }

    add_benchmark(pings, name);
}


void track_error_benchmark::print_summary()
{
    for (const pair<string, double>& p : track_rms_errors) {
        cout << p.first << " RMS track error: " << p.second << endl;
    }
    for (const pair<string, double>& p : consistency_rms_errors) {
        cout << p.first << " RMS consistency error: " << p.second << endl;
    }
    for (const pair<string, string>& p : error_img_paths) {
        cout << p.first << " consistency image path: " << p.second << endl;
    }
}

