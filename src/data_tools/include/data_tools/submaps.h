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

#ifndef SUBMAPS_H
#define SUBMAPS_H

#include <Eigen/Dense>
#include <iostream>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

namespace submaps {

using ObsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;
using SubmapsT = std::vector<std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >;
using MatchesT = std::vector<std::pair<int, int> >; // tells us which maps overlap
using ConstraintsT = std::vector<std::tuple<int, int, Eigen::Vector3d, Eigen::Vector3d> >;
using BBsT = std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> >;
using TransTT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsTT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;

Eigen::MatrixXd read_submap(const boost::filesystem::path& filename);
SubmapsT read_submaps(const boost::filesystem::path& folder);
MatchesT compute_matches(const TransTT& trans, const RotsTT& rots, const BBsT& bounds);
ConstraintsT compute_binary_constraints(const TransTT& trans, const RotsTT& rots, const ObsT& points);
ConstraintsT compute_binary_constraints(const TransTT& trans, const RotsTT& rots, const ObsT& points, const ObsT& tracks);
Eigen::MatrixXd get_points_in_bound_transform(Eigen::MatrixXd points, Eigen::Vector3d& t,
				                              Eigen::Matrix3d& R, Eigen::Vector3d& t_in,
											  Eigen::Matrix3d& R_in, double bound);
Eigen::MatrixXd get_points_in_bound_transform(Eigen::MatrixXd points, Eigen::Vector3d& t,
				                              Eigen::Matrix3d& R, Eigen::Vector3d& t_in,
											  Eigen::Matrix3d& R_in, Eigen::Matrix2d& bounds);

template <typename ProcessT>
std::tuple<Eigen::Vector3d, Eigen::Matrix3d> train_gp(Eigen::MatrixXd& points, ProcessT& gp)
{
    std::cout << "Training gaussian process..." << std::endl;
	double meanx = points.col(0).mean();
	double meany = points.col(1).mean();
	double meanz = points.col(2).mean();
	
	points.col(0).array() -= meanx;
	points.col(1).array() -= meany;
	points.col(2).array() -= meanz;

    Eigen::MatrixXd X = points.leftCols(2);
	Eigen::VectorXd y = points.col(2);
	//gp.train_parameters(X, y);
	gp.add_measurements(X, y);

	std::cout << "Done training gaussian process..." << std::endl;

	Eigen::Vector3d t(meanx, meany, meanz);
	Eigen::Matrix3d R;
	R.setIdentity();

    return std::make_tuple(t, R);
}

} // namespace submaps

#endif // SUBMAPS_H
