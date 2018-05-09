// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)

#include <fstream>
#include <iostream>
#include <string>
#include <cxxopts.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <ceres/ceres.h>
#include "common/read_g2o.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pose_graph_3d/pose_graph_3d_error_term.h>
#include <pose_graph_3d/types.h>

#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/rbf_kernel.h>
#include <sparse_gp/gaussian_noise.h>

#include <data_tools/colormap.h>
#include <data_tools/submaps.h>

#include <eigen_cereal/eigen_cereal.h>
#include <cereal/archives/json.hpp>
#include <cereal/types/vector.hpp>

using namespace std;
using TransT = vector<vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > >;
using RotsT = vector<vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > >;
using ProcessT = sparse_gp<rbf_kernel, gaussian_noise>;
using SubmapsGPT = vector<vector<ProcessT> >;
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");

namespace ceres {
namespace examples {

    // first template argument should be the dimension of the residual
	// the second argument should be the number of parameters
    class GaussianProcessCostFunction : public ceres::SizedCostFunction<1, 6> {
    public:
        virtual ~GaussianProcessCostFunction() {}
        virtual bool Evaluate(double const* const* parameters,
            double* residuals,
            double** jacobians) const
        {
            const double x = parameters[0][0];
            residuals[0] = 10 - x;

            // Compute the Jacobian if asked for.
            if (jacobians != NULL && jacobians[0] != NULL) {
                jacobians[0][0] = -1;
            }
            return true;
        }
    };

    // Constructs the nonlinear least squares optimization problem from the pose
    // graph constraints.
    void build_optimization_problem(SubmapsT& submaps, SubmapsGPT& submap_gps,
					                TransT& trans, RotsT& rots, ceres::Problem* problem)
    {
        CHECK(problem != NULL);

        ceres::LossFunction* loss_function = NULL;
        ceres::LocalParameterization* quaternion_local_parameterization = new EigenQuaternionParameterization;

		/*
        for (VectorOfConstraints::const_iterator constraints_iter = constraints.begin();
             constraints_iter != constraints.end(); ++constraints_iter) {
            const Constraint3d& constraint = *constraints_iter;

            MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
            CHECK(pose_begin_iter != poses->end())
                << "Pose with ID: " << constraint.id_begin << " not found.";
            MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
            CHECK(pose_end_iter != poses->end())
                << "Pose with ID: " << constraint.id_end << " not found.";

            const Eigen::Matrix<double, 6, 6> sqrt_information = constraint.information.llt().matrixL();
            // Ceres will take ownership of the pointer.
            ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

            problem->AddResidualBlock(cost_function, loss_function,
                pose_begin_iter->second.p.data(),
                pose_begin_iter->second.q.coeffs().data(),
                pose_end_iter->second.p.data(),
                pose_end_iter->second.q.coeffs().data());

            problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                quaternion_local_parameterization);
            problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                quaternion_local_parameterization);
        }
		*/

        // The pose graph optimization problem has six DOFs that are not fully
        // constrained. This is typically referred to as gauge freedom. You can apply
        // a rigid body transformation to all the nodes and the optimization problem
        // will still have the exact same cost. The Levenberg-Marquardt algorithm has
        // internal damping which mitigates this issue, but it is better to properly
        // constrain the gauge freedom. This can be done by setting one of the poses
        // as constant so the optimizer cannot change it.
		/*
        MapOfPoses::iterator pose_start_iter = poses->begin();
        CHECK(pose_start_iter != poses->end()) << "There are no poses.";
        problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
        problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
		*/
    }

    // Returns true if the solve was successful.
    bool SolveOptimizationProblem(ceres::Problem* problem)
    {
        CHECK(problem != NULL);

        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

        ceres::Solver::Summary summary;
        ceres::Solve(options, problem, &summary);

        std::cout << summary.FullReport() << '\n';

        return summary.IsSolutionUsable();
    }

    // Output the poses to the file with format: id x y z q_x q_y q_z q_w.
    bool OutputPoses(const std::string& filename, const MapOfPoses& poses)
    {
        std::fstream outfile;
        outfile.open(filename.c_str(), std::istream::out);
        if (!outfile) {
            LOG(ERROR) << "Error opening the file: " << filename;
            return false;
        }
        for (std::map<int, Pose3d, std::less<int>,
                 Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
                 const_iterator poses_iter
             = poses.begin();
             poses_iter != poses.end(); ++poses_iter) {
            const std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
                value_type& pair
                = *poses_iter;
            outfile << pair.first << " " << pair.second.p.transpose() << " "
                    << pair.second.q.x() << " " << pair.second.q.y() << " "
                    << pair.second.q.z() << " " << pair.second.q.w() << '\n';
        }
        return true;
    }

} // namespace examples
} // namespace ceres

/*
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    CERES_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

    ceres::examples::MapOfPoses poses;
    ceres::examples::VectorOfConstraints constraints;

    CHECK(ceres::examples::ReadG2oFile(FLAGS_input, &poses, &constraints))
        << "Error reading the file: " << FLAGS_input;

    std::cout << "Number of poses: " << poses.size() << '\n';
    std::cout << "Number of constraints: " << constraints.size() << '\n';

    CHECK(ceres::examples::OutputPoses("poses_original.txt", poses))
        << "Error outputting to poses_original.txt";

    ceres::Problem problem;
    ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);

    CHECK(ceres::examples::SolveOptimizationProblem(&problem))
        << "The solve was not successful, exiting.";

    CHECK(ceres::examples::OutputPoses("poses_optimized.txt", poses))
        << "Error outputting to poses_original.txt";

    return 0;
}
*/

tuple<SubmapsGPT, TransT, RotsT> train_gps(SubmapsT& submaps, double lsq, double sigma, double s0)
{
    if (boost::filesystem::exists("gp_submaps.json")) {
		TransT trans;
		RotsT rots;
		SubmapsGPT gps;
        std::ifstream is("gp_submaps.json");
        {
			cereal::JSONInputArchive archive(is);
			archive(gps, trans, rots);
        }
        is.close();
		return make_tuple(gps, trans, rots);
    }

    // check if already available
    TransT trans(submaps.size());
	RotsT rots(submaps.size());
	SubmapsGPT gps(submaps.size());
    for (int i = 0; i < submaps.size(); ++i) {
		trans[i].resize(submaps[i].size());
		rots[i].resize(submaps[i].size());
        for (int j = 0; j < submaps[i].size(); ++i) {
			ProcessT gp(100, s0);
			gp.kernel.sigmaf_sq = sigma;
			gp.kernel.l_sq = lsq*lsq;
			gp.kernel.p(0) = gp.kernel.sigmaf_sq;
			gp.kernel.p(1) = gp.kernel.l_sq;
			tie(trans[i][j], rots[i][j]) = train_gp(submaps[i][j], gp);
			gps[i].push_back(gp);
        }
    }
	// write to disk for next time
    std::ofstream os("gp_submaps.json");
	{
		cereal::JSONOutputArchive archive(os);
		archive(gps, trans, rots);
	}
    os.close();
    
	return make_tuple(gps, trans, rots);
}

tuple<TransT, RotsT> distort_transforms(const TransT& trans, const RotsT& rots)
{

}

int main(int argc, char** argv)
{
    string folder_str;
	double lsq = 100.;
	double sigma = 1.;
	double s0 = 1.;

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	//options.positional_help("[optional args]").show_positional_help();
	options.add_options()
      ("help", "Print help")
      ("folder", "Folder", cxxopts::value(folder_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("s0", "Measurement noise", cxxopts::value(s0));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("folder") == 0) {
		cout << "Please provide folder arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path folder(folder_str);
	cout << "Folder : " << folder << endl;

    SubmapsT submaps = read_submaps(folder);
	SubmapsGPT submap_gps;
	TransT trans;
	RotsT rots;

	// NOTE: this function checks if the file submap_gps.json is available
	// If it is, it will use the already trained gps. If you're using new
	// parameters, delete that file
	tie(submap_gps, trans, rots) = train_gps(submaps, lsq, sigma, s0);

	tie(trans, rots) = distort_transforms(trans, rots);
    
	ceres::Problem problem;
    ceres::examples::build_optimization_problem(submaps, submap_gps, trans, rots, &problem);
    
	CHECK(ceres::examples::SolveOptimizationProblem(&problem))
        << "The solve was not successful, exiting.";

    ceres::examples::MapOfPoses poses;
    CHECK(ceres::examples::OutputPoses("poses_optimized.txt", poses))
        << "Error outputting to poses_original.txt";

	return 0;
}
