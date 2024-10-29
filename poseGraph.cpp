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

    // Add the prior
    if (_prior) {

        std::vector<double *> prior_parameter_blocks;
        for (auto &nf_prior : _prior->nf_idx_map) {

            // Check if the nf are not in the parameters
            if (nf_pose_map.find(nf_prior.first) == nf_pose_map.end()) {
                nf_pose_map.emplace(nf_prior.first, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
                problem.AddParameterBlock(nf_pose_map.at(nf_prior.first).values(), 6);
            }

            prior_parameter_blocks.push_back(nf_pose_map.at(nf_prior.first).values());
        }

        ceres::CostFunction *cost_fct = new MarginalizationPrior(_prior->J, _prior->r, _prior->nf_idx_map);
        problem.AddResidualBlock(cost_fct, loss_function, prior_parameter_blocks);
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

void PoseGraph::marginalize(std::shared_ptr<NavFrame> nf) {

    // Create a marginalization scheme
    isae::Marginalization marg_sch;
    std::unordered_map<std::shared_ptr<NavFrame>, isae::PoseParametersBlock> nf_pose_map;
    std::unordered_map<std::shared_ptr<NavFrame>, int> nf_idx_map;

    // nf is the variable to marg
    marg_sch._m  = 6;
    marg_sch._n  = 0;
    int last_idx = 0;
    nf_pose_map.emplace(nf, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
    nf_idx_map.emplace(nf, last_idx);
    last_idx += 6;

    // Check for variables to set prior on
    for (auto nf_relfact : _nf_relfact_map) {
        RelativePoseFactor relfact = nf_relfact.second;

        if (relfact.nf_a == nf || relfact.nf_b == nf) {

            // Check if the nf are not in the parameters
            if (nf_pose_map.find(relfact.nf_a) == nf_pose_map.end()) {
                nf_pose_map.emplace(relfact.nf_a, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
                nf_idx_map.emplace(relfact.nf_a, last_idx);
                last_idx += 6;
                marg_sch._n += 6;
            }

            if (nf_pose_map.find(relfact.nf_b) == nf_pose_map.end()) {
                nf_pose_map.emplace(relfact.nf_b, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
                nf_idx_map.emplace(relfact.nf_b, last_idx);
                last_idx += 6;
                marg_sch._n += 6;
            }
        }
    }

    if (_prior) {
        for (auto &nf_prior : _prior->nf_idx_map) {
            // Check if the nf are not in the parameters
            if (nf_pose_map.find(nf_prior.first) == nf_pose_map.end()) {
                nf_pose_map.emplace(nf_prior.first, isae::PoseParametersBlock(Eigen::Affine3d::Identity()));
                nf_idx_map.emplace(nf_prior.first, last_idx);
                last_idx += 6;
                marg_sch._n += 6;
            }
        }
    }

    // Check if there is a position factor
    if (_nf_absfact_map.find(nf) != _nf_absfact_map.end()) {
        AbsolutePositionFactor nf_absfact = _nf_absfact_map.at(nf);

        std::vector<double *> parameter_blocks;
        std::vector<int> parameter_idx;

        parameter_idx.push_back(nf_idx_map.at(nf));
        parameter_blocks.push_back(nf_pose_map.at(nf).values());

        ceres::CostFunction *cost_fct = new PositionPrior(nf->_T_n_f, nf_absfact.t, nf_absfact.inf);
        marg_sch.addMarginalizationBlock(
            std::make_shared<isae::MarginalizationBlockInfo>(cost_fct, parameter_idx, parameter_blocks));
    }

    // Check if there is a position factor
    if (_nf_abspose_map.find(nf) != _nf_abspose_map.end()) {
        AbsolutePoseFactor nf_abspose = _nf_abspose_map.at(nf);

        std::vector<double *> parameter_blocks;
        std::vector<int> parameter_idx;

        parameter_idx.push_back(nf_idx_map.at(nf));
        parameter_blocks.push_back(nf_pose_map.at(nf).values());

        ceres::CostFunction *cost_fct = new PosePriordx(nf->_T_n_f, nf_abspose.T, nf_abspose.inf);
        marg_sch.addMarginalizationBlock(
            std::make_shared<isae::MarginalizationBlockInfo>(cost_fct, parameter_idx, parameter_blocks));
    }

    // Check if there is a relative pose factor
    std::vector<std::shared_ptr<NavFrame>> nfs_to_rm; // nf in the map to remove factors
    for (auto nf_relfact : _nf_relfact_map) {
        RelativePoseFactor relfact = nf_relfact.second;

        if (relfact.nf_a == nf || relfact.nf_b == nf) {
            nfs_to_rm.push_back(nf_relfact.first);

            Eigen::Affine3d T_n_a         = relfact.nf_a->_T_n_f;
            Eigen::Affine3d T_n_b         = relfact.nf_b->_T_n_f;
            ceres::CostFunction *cost_fct = new Relative6DPose(T_n_a, T_n_b, relfact.T_a_b, relfact.inf);

            std::vector<double *> parameter_blocks;
            std::vector<int> parameter_idx;

            parameter_idx.push_back(nf_idx_map.at(relfact.nf_a));
            parameter_idx.push_back(nf_idx_map.at(relfact.nf_b));
            parameter_blocks.push_back(nf_pose_map.at(relfact.nf_a).values());
            parameter_blocks.push_back(nf_pose_map.at(relfact.nf_b).values());

            marg_sch.addMarginalizationBlock(
                std::make_shared<isae::MarginalizationBlockInfo>(cost_fct, parameter_idx, parameter_blocks));
        }
    }

    // Add prior factor
    if (_prior) {
        std::vector<double *> parameter_blocks;
        std::vector<int> parameter_idx;
        for (auto &nf_prior : _prior->nf_idx_map) {

            parameter_idx.push_back(nf_idx_map.at(nf_prior.first));
            parameter_blocks.push_back(nf_pose_map.at(nf_prior.first).values());
        }
        ceres::CostFunction *cost_fct = new MarginalizationPrior(_prior->J, _prior->r, _prior->nf_idx_map);
        marg_sch.addMarginalizationBlock(
            std::make_shared<isae::MarginalizationBlockInfo>(cost_fct, parameter_idx, parameter_blocks));
    }

    // Compute the prior
    marg_sch.computeSchurComplement();
    marg_sch.computeJacobiansAndResiduals();

    // Clean the maps
    _nf_absfact_map.erase(nf);
    for (auto &nf_to_rm : nfs_to_rm)
        _nf_relfact_map.erase(nf_to_rm);
    _nf_abspose_map.erase(nf);

    // Update the indices
    std::unordered_map<std::shared_ptr<NavFrame>, int> nf_idx_map_up;
    for (auto &nf_idx : nf_idx_map) {
        if (nf_idx.second >= marg_sch._m) {
            nf_idx_map_up.emplace(nf_idx.first, nf_idx.second - marg_sch._m);
            std::cout << nf_idx.second - marg_sch._m << std::endl;
        }
    }

    // Update the prior
    if (!_prior) {
        MarginalizationFactor mf;
        mf.J          = marg_sch._marginalization_jacobian;
        mf.r          = marg_sch._marginalization_residual;
        mf.nf_idx_map = nf_idx_map_up;
        _prior        = std::make_shared<MarginalizationFactor>(mf);
    } else {
        _prior->J          = marg_sch._marginalization_jacobian;
        _prior->r          = marg_sch._marginalization_residual;
        _prior->nf_idx_map = nf_idx_map_up;
    }
}