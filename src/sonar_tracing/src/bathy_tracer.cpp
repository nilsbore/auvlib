#include <sonar_tracing/bathy_tracer.h>
#include <chrono>
#include <igl/barycentric_to_global.h>

using namespace std;

const bool DEBUG_OUTPUT = false;

Eigen::MatrixXd BathyTracer::ray_mesh_intersection(
    const Eigen::MatrixXd& V_source,
    const Eigen::MatrixXd& N_source,
    const Eigen::MatrixXd& V_target,
    const Eigen::MatrixXi& F_target)
{
    if (first_V != V_target.row(0).transpose() ||
        first_F != F_target.row(0).transpose()) {
        embree.init(V_target.template cast<float>(),F_target.template cast<int>());
        first_V = V_target.row(0).transpose();
        first_F = F_target.row(0).transpose();
    }

    double tol = 0.00001;

    // Allocate matrix for the result
    Eigen::MatrixXd R;
    R.resize(V_source.rows(), 3);

    // Shoot rays from the source to the target
    for (unsigned i = 0; i < V_source.rows(); ++i) {
        igl::Hit hit;

        // Shoot ray
        Eigen::RowVector3d pos = V_source.row(i) - tol * N_source.row(i);
        Eigen::RowVector3d dir = N_source.row(i);
        bool did_hit = embree.intersectBeam(pos.cast<float>(), dir.cast<float>(), hit);

        if (did_hit) {
            R.row(i) << hit.id, hit.u, hit.v;
        }
        else {
            R.row(i) << -1, 0, 0;
        }
    }

    return R;
}

double BathyTracer::depth_mesh_underneath_vehicle(const Eigen::Vector3d& origin,
                                                  const Eigen::MatrixXd& V_target,
                                                  const Eigen::MatrixXi& F_target)
{
    if (first_V != V_target.row(0).transpose() ||
        first_F != F_target.row(0).transpose()) {
        embree.init(V_target.template cast<float>(),F_target.template cast<int>());
        first_V = V_target.row(0).transpose();
        first_F = F_target.row(0).transpose();
    }

    // Shoot ray
    igl::Hit hit;
    bool did_hit = embree.intersectBeam(origin.cast<float>(), Eigen::Vector3f(0., 0., -1.), hit);
    return did_hit? origin(2) - V_target(F_target(hit.id, 0), 2) : 0.;
}

tuple<Eigen::MatrixXd, Eigen::MatrixXi> BathyTracer::compute_hits(const Eigen::Vector3d& sensor_origin, const Eigen::MatrixXd& dirs, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    auto start = chrono::high_resolution_clock::now();
    igl::Hit hit;
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "compute_sss_dirs time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    Eigen::MatrixXd hits(dirs.rows(), 3);
    Eigen::VectorXi hits_inds(dirs.rows());

    int nbr_lines = dirs.rows();
    Eigen::MatrixXd P = Eigen::MatrixXd(nbr_lines, 3);

    // TODO: we can easily split origin into origin_port and origin_stbd
    P.rowwise() = sensor_origin.transpose();
    Eigen::MatrixXd hits_info = ray_mesh_intersection(P, dirs, V, F);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "line_mesh_intersection time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    Eigen::MatrixXd global_hits = igl::barycentric_to_global(V, F, hits_info);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "barycentric_to_global time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    int hit_count = 0;
    for (int i = 0; i < dirs.rows(); ++i) {
        int hit = hits_info(i, 0);
        if (hit > 0) {
            int vind = F(hit, 0);
            hits_inds(hit_count) = hit;
            hits.row(hit_count) = global_hits.row(i);
            ++hit_count;
        }
    }
    hits.conservativeResize(hit_count, 3);
    hits_inds.conservativeResize(hit_count);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "hits loop time: " << duration.count() << " microseconds" << endl;

    return make_tuple(hits, hits_inds);
}
