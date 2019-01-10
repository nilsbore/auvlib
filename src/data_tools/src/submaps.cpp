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

#include <data_tools/submaps.h>
#include <data_tools/colormap.h>

#include <sstream>
#include <fstream>
#include <iomanip>

using namespace std;

namespace submaps {

Eigen::MatrixXd read_submap(const boost::filesystem::path& filename)
{
	Eigen::MatrixXd points;
	points.resize(90000, 3);
	std::ifstream infile(filename.string());
	std::string line;
	int i = 0;
	int counter = 0;
	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		double x, y, z;
		if (!(iss >> x >> y >> z)) {
			break;
		} // error
		if (counter % 37 != 0) {
		    ++counter;
		    continue;
		}
		if (i >= points.rows()) {
			points.conservativeResize(points.rows() + 90000, 3);
		}
		points.row(i) << x, y, z;
		++counter;
		++i;
	}
	points.conservativeResize(i, 3);
	return points;
}

SubmapsT read_submaps(const boost::filesystem::path& folder)
{
    SubmapsT submaps;

	bool should_break = false;
	for (int ii = 0; !should_break; ++ii) {
		for (int jj = 0; ; ++jj) {
			stringstream ss;
			ss << "patch_";
			ss << setfill('0') << setw(2) << ii << "_";
			ss << setfill('0') << setw(2) << jj;
			ss << ".xyz";
			boost::filesystem::path filename = folder / ss.str();
			cout << "Processing " << filename.string() << endl;
			if (!boost::filesystem::exists(filename)) {
				if (jj == 0) {
					should_break = true;
				}
				break;
			}
			Eigen::MatrixXd points = read_submap(filename);
			if (jj == 0) {
                submaps.push_back(vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >());
            }
			submaps.back().push_back(points);
		}
	}

	return submaps;
}

// first stupid attempt
//
// Check if these points are within the other:
//
//  ----------------
//  |              |
//  |   x      x   |
//  |              |
//  |              |
//  |   x      x   |
//  |              |
//  ----------------
//
MatchesT compute_matches(const TransTT& trans, const RotsTT& rots, const BBsT& bounds)
{
    MatchesT matches;

    // compare each submap with every other submap
    for (int i = 0; i < trans.size(); ++i) {
        for (int j = 0; j < i; ++j) {
            //double area1 = (bounds[i](1, 1) - bounds[i](0, 1))*(bounds[i](1, 0) - bounds[i](0, 0));
            //double area2 = (bounds[j](1, 1) - bounds[j](0, 1))*(bounds[j](1, 0) - bounds[j](0, 0));
            Eigen::Matrix2d bb1 = bounds[i]; Eigen::Matrix2d bb2 = bounds[2];
            Eigen::Matrix<double, 4, 3> corners1, corners2;
            corners1.setZero(); corners2.setZero();
            corners1.topLeftCorner<2, 2>() = bb1;
            corners1.block<2, 1>(2, 0) = bb1.block<2, 1>(0, 0);
            corners1(2, 1) = corners1(1, 1);
            corners1(3, 1) = corners1(0, 1);
            corners2.topLeftCorner<2, 2>() = bb2;
            corners2.block<2, 1>(2, 0) = bb2.block<2, 1>(0, 0);
            corners2(2, 1) = corners2(1, 1);
            corners2(3, 1) = corners2(0, 1);
            corners1 = ((0.5*corners1*rots[i].transpose()).rowwise() + (trans[i] - trans[j]).transpose())*rots[j];
            corners2 = ((0.5*corners2*rots[j].transpose()).rowwise() + (trans[j] - trans[i]).transpose())*rots[i];
            bool match = false;
            for (int i = 0; i < 4; ++i) {
                if (corners1(i, 0) < bb2(1, 0) && corners1(i, 0) > bb2(0, 0) &&
                    corners1(i, 1) < bb2(1, 1) && corners1(i, 1) > bb2(0, 1)) {
                    match = true;
                    break;
                }
                if (corners2(i, 0) < bb1(1, 0) && corners2(i, 0) > bb1(0, 0) &&
                    corners2(i, 1) < bb1(1, 1) && corners2(i, 1) > bb1(0, 1)) {
                    match = true;
                    break;
                }
            }
            if (match) {
                matches.push_back(make_pair(i, j));
            }
        }
    }

    return matches;
}

// NOTE: we need the poses here
ConstraintsT compute_binary_constraints(const TransTT& trans, const RotsTT& rots, const ObsT& points)
{
    const int swath_width = 512;

    ConstraintsT binary_constraints;

    for (int i = 1; i < points.size(); ++i) {

        Eigen::Vector3d last_point1 = points[i-1].row(points[i-1].rows()-swath_width/2).transpose();
        Eigen::Vector3d first_point2 = points[i].row(swath_width/2).transpose();
        Eigen::Vector3d last_point1_transformed = rots[i-1]*last_point1 + trans[i-1];
        Eigen::Vector3d first_point2_transformed = rots[i]*first_point2 + trans[i];
        first_point2 = rots[i].transpose()*(last_point1_transformed - trans[i]);
        double dist = (first_point2_transformed-last_point1_transformed).norm();
        cout << "Distance between " << i-1 << " and " << i << ": " << dist << endl;
        if (dist < 0.4) {
            cout << "Found a binary constraint between" << i-1 << " and " << i << endl;
            binary_constraints.push_back(make_tuple(i-1, i, last_point1, first_point2));
            // TODO: This is debug code, remove afterwards
            //binary_constraints.push_back(make_tuple(i-1, i, rots[i-1]*last_point1, rots[i]*first_point2));
        }
    }

    return binary_constraints;
}

// NOTE: we need the poses here
ConstraintsT compute_binary_constraints(const TransTT& trans, const RotsTT& rots, const ObsT& points, const ObsT& tracks)
{
    const int swath_width = 512;

    ConstraintsT binary_constraints;

    for (int i = 1; i < points.size(); ++i) {

        Eigen::Vector3d last_point1 = tracks[i-1].bottomRows<1>().transpose();
        Eigen::Vector3d first_point2 = tracks[i].topRows<1>().transpose();
        Eigen::Vector3d last_point1_transformed = rots[i-1]*last_point1 + trans[i-1];
        Eigen::Vector3d first_point2_transformed = rots[i]*first_point2 + trans[i];
        first_point2 = rots[i].transpose()*(last_point1_transformed - trans[i]);
        double dist = (first_point2_transformed-last_point1_transformed).norm();
        cout << "Distance between " << i-1 << " and " << i << ": " << dist << endl;
        if (dist < 20.) {
            cout << "Found a binary constraint between" << i-1 << " and " << i << endl;
            binary_constraints.push_back(make_tuple(i-1, i, last_point1, first_point2));
            // TODO: This is debug code, remove afterwards
            //binary_constraints.push_back(make_tuple(i-1, i, rots[i-1]*last_point1, rots[i]*first_point2));
        }
    }

    return binary_constraints;
}

Eigen::MatrixXd get_points_in_bound_transform(Eigen::MatrixXd points, Eigen::Vector3d& t,
				                              Eigen::Matrix3d& R, Eigen::Vector3d& t_in,
											  Eigen::Matrix3d& R_in, double bound)
{
    points *= R.transpose()*R_in;
	points.rowwise() += (t.transpose()*R_in - t_in.transpose()*R_in);

    int counter = 0;
	for (int i = 0; i < points.rows(); ++i) {
		if (points(i, 0) < bound && points(i, 0) > -bound &&
		    points(i, 1) < bound && points(i, 1) > -bound) {
		    points.row(counter) = points.row(i);
			++counter;
		}
    }
	points.conservativeResize(counter, 3);
	return points;
}

Eigen::MatrixXd get_points_in_bound_transform(Eigen::MatrixXd points, Eigen::Vector3d& t,
				                              Eigen::Matrix3d& R, Eigen::Vector3d& t_in,
											  Eigen::Matrix3d& R_in, Eigen::Matrix2d& bounds)
{
    points *= R.transpose()*R_in;
	points.rowwise() += (t.transpose()*R_in - t_in.transpose()*R_in);

    int counter = 0;
	for (int i = 0; i < points.rows(); ++i) {
		if (points(i, 0) < bounds(1, 0) && points(i, 0) > bounds(0, 0) &&
		    points(i, 1) < bounds(1, 1) && points(i, 1) > bounds(0, 1)) {
		    points.row(counter) = points.row(i);
			++counter;
		}
    }
	points.conservativeResize(counter, 3);
	return points;
}

}
