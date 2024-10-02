#include "poseGraph.hpp"

void PoseGraph::solveGraph() {

    // Build the ceres problem
    ceres::Problem problem;
    ceres::LossFunction *loss_function = nullptr;

    std::unordered_map<std::shared_ptr<NavFrame>, isae::PoseParametersBlock> nf_pose_map;

    // Add absolute pose constraints
    for (auto &nf_absfact : _nf_abspose_map) {
        nf_pose_map.emplace(nf_absfact.first, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
        problem.AddParameterBlock(nf_pose_map.at(nf_absfact.first).values(), 6);

        ceres::CostFunction *cost_fct =
            new PosePriordx(nf_absfact.first->_T_n_f, nf_absfact.second.T, nf_absfact.second.inf);

        problem.AddResidualBlock(cost_fct, loss_function, nf_pose_map.at(nf_absfact.second.nf).values());
    }

    // Add absolute position constraints
    for (auto &nf_absfact : _nf_absfact_map) {
        if (nf_pose_map.find(nf_absfact.first) == nf_pose_map.end()) {
            nf_pose_map.emplace(nf_absfact.first, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
            problem.AddParameterBlock(nf_pose_map.at(nf_absfact.first).values(), 6);
        }

        ceres::CostFunction *cost_fct =
            new PositionPrior(nf_absfact.first->_T_n_f, nf_absfact.second.t, nf_absfact.second.inf);

        problem.AddResidualBlock(cost_fct, loss_function, nf_pose_map.at(nf_absfact.second.nf).values());
    }

    // Add all constraints
    for (auto &nf_relfact : _nf_relfact_map) {

        // Check if the nf are not in the parameters
        if (nf_pose_map.find(nf_relfact.second.nf_a) == nf_pose_map.end()) {
            nf_pose_map.emplace(nf_relfact.second.nf_a, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
            problem.AddParameterBlock(nf_pose_map.at(nf_relfact.second.nf_a).values(), 6);
        }

        if (nf_pose_map.find(nf_relfact.second.nf_b) == nf_pose_map.end()) {
            nf_pose_map.emplace(nf_relfact.second.nf_b, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
            problem.AddParameterBlock(nf_pose_map.at(nf_relfact.second.nf_b).values(), 6);
        }

        Eigen::Affine3d T_n_a = nf_relfact.second.nf_a->_T_n_f;
        Eigen::Affine3d T_n_b = nf_relfact.second.nf_b->_T_n_f;
        ceres::CostFunction *cost_fct =
            new Relative6DPose(T_n_a, T_n_b, nf_relfact.second.T_a_b, nf_relfact.second.inf);

        problem.AddResidualBlock(cost_fct,
                                 loss_function,
                                 nf_pose_map.at(nf_relfact.second.nf_a).values(),
                                 nf_pose_map.at(nf_relfact.second.nf_b).values());
    }

    // Solve the problem we just built
    ceres::Solver::Options options;
    options.trust_region_strategy_type         = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type                 = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations                 = 20;
    options.minimizer_progress_to_stdout       = false;
    options.use_explicit_schur_complement      = true;
    options.function_tolerance                 = 1e-3;
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.num_threads                        = 4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    // Update the poses
    for (auto &nf_pose : nf_pose_map) {
        Eigen::Affine3d T_init = nf_pose.first->_T_n_f;
        nf_pose.first->_T_n_f  = T_init * nf_pose.second.getPose();
    }
}