#include "poseGraph.hpp"

void PoseGraph::addPose(unsigned long long ts, Eigen::Affine3d &pose) { _nodes_map.emplace(ts, pose); }

void PoseGraph::addEdge(unsigned long long ts_a,
                        unsigned long long ts_b,
                        const Eigen::Affine3d &T_a_b,
                        const Eigen::MatrixXd &inf) {
    PoseFactor pf;
    pf.ts_a  = ts_a;
    pf.ts_b  = ts_b;
    pf.T_a_b = T_a_b;
    pf.inf   = inf;
    _edge_constraints.push_back(pf);
}

unsigned int PoseGraph::numNodes() { return _nodes_map.size(); }

unsigned int PoseGraph::numEdges() { return _edge_constraints.size(); }

std::vector<std::pair<unsigned long long, Eigen::Affine3d>> PoseGraph::getNodes() {

    // Copy keys to a vector
    std::vector<unsigned long long> keys;
    for (const auto& pair : _nodes_map) {
        keys.push_back(pair.first);
    }

    // Sort the vector
    std::sort(keys.begin(), keys.end());

    // Fill the vector
    std::vector<std::pair<unsigned long long, Eigen::Affine3d>> node_vec;
    for (auto key : keys) {
        node_vec.push_back({key, _nodes_map.at(key)});
    }


    return node_vec;
}

void PoseGraph::solveGraph() {

    // Build the ceres problem
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(std::sqrt(1.345));

    std::unordered_map<unsigned long long, isae::PoseParametersBlock> ts_pose_map;

    for (auto ts_pose : _nodes_map) {
        ts_pose_map.emplace(ts_pose.first, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
        problem.AddParameterBlock(ts_pose_map.at(ts_pose.first).values(), 6);

        // Solve the gauge freedom
        if (ts_pose.first == _ts_gauge)
            problem.SetParameterBlockConstant(ts_pose_map.at(ts_pose.first).values());
    }

    // Add all constraints
    for (auto constraint : _edge_constraints) {
        Eigen::Affine3d T_w_a         = _nodes_map.at(constraint.ts_a);
        Eigen::Affine3d T_w_b         = _nodes_map.at(constraint.ts_b);
        ceres::CostFunction *cost_fct = new Relative6DPose(T_w_a, T_w_b, constraint.T_a_b, constraint.inf);

        problem.AddResidualBlock(cost_fct,
                                 loss_function,
                                 ts_pose_map.at(constraint.ts_a).values(),
                                 ts_pose_map.at(constraint.ts_b).values());

    }

    // Solve the problem we just built
    ceres::Solver::Options options;
    options.trust_region_strategy_type         = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type                 = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations                 = 500;
    options.minimizer_progress_to_stdout       = false;
    options.use_explicit_schur_complement      = true;
    options.function_tolerance                 = 1e-3;
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.num_threads                        = 4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    // Update the poses 
    for (auto &ts_pose : ts_pose_map) {
        Eigen::Affine3d T_init = _nodes_map.at(ts_pose.first);
        _nodes_map.at(ts_pose.first) = T_init * ts_pose.second.getPose();
    }


}