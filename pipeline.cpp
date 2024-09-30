#include "pipeline.hpp"

const Eigen::Vector3d Pipeline::llhToEcef(const Eigen::Vector3d &llh) {
    double Sp = std::sin(llh.x() * deg2rad);
    double Cp = std::cos(llh.x() * deg2rad);
    double Sl = std::sin(llh.y() * deg2rad);
    double Cl = std::cos(llh.y() * deg2rad);
    double N  = _a / std::sqrt(1 - _e2 * Sp * Sp);
    Eigen::Vector3d out;
    out.x() = (N + llh.z()) * Cp * Cl;
    out.y() = (N + llh.z()) * Cp * Sl;
    out.z() = (N * (1 - _e2) + llh.z()) * Sp;
    return out;
}

const Eigen::Vector3d Pipeline::ecefToENU(const Eigen::Vector3d &ecef) {
    Eigen::Vector3d dx = ecef - _ecef_ref;
    return _R_n_e * dx;
}

void Pipeline::setRef(const Eigen::Vector3d &llh_ref) {
    _llh_ref  = llh_ref;
    _ecef_ref = llhToEcef(_llh_ref);

    // Compute projection matrix PM_ used to project coordinates in LTP
    _R_n_e(0, 0) = -std::sin(_llh_ref.y() * deg2rad);
    _R_n_e(0, 1) = +std::cos(_llh_ref.y() * deg2rad);
    _R_n_e(0, 2) = 0.0f;

    _R_n_e(1, 0) = -std::sin(_llh_ref.x() * deg2rad) * std::cos(_llh_ref.y() * deg2rad);
    _R_n_e(1, 1) = -std::sin(_llh_ref.x() * deg2rad) * std::sin(_llh_ref.y() * deg2rad);
    _R_n_e(1, 2) = std::cos(_llh_ref.x() * deg2rad);

    _R_n_e(2, 0) = std::cos(_llh_ref.x() * deg2rad) * std::cos(_llh_ref.y() * deg2rad);
    _R_n_e(2, 1) = std::cos(_llh_ref.x() * deg2rad) * std::sin(_llh_ref.y() * deg2rad);
    _R_n_e(2, 2) = std::sin(_llh_ref.x() * deg2rad);
}

std::shared_ptr<NavFrame> Pipeline::next() {
    std::shared_ptr<NavFrame> nf = std::shared_ptr<NavFrame>(new NavFrame());

    while (_nf_queue.empty())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    nf = _nf_queue.front();
    _nf_queue.pop();

    return nf;
}

void Pipeline::init() {

    // Init SLAM
    while (!_slam->_is_init) {
        _nf = next();
        _slam->_slam_param->getDataProvider()->addFrameToTheQueue(_nf->_frame);
    }

    // Wait for a frame with GPS
    while (_nf->_gnss_meas == nullptr) {
        _nf = next();
        _slam->_slam_param->getDataProvider()->addFrameToTheQueue(_nf->_frame);
    }

    // Get the ouput from the SLAM
    std::shared_ptr<isae::Frame> frame_ready = _slam->_frame_to_display;
    while (frame_ready != _nf->_frame) {
        frame_ready = _slam->_frame_to_display;
    }

    // Save the pose in the SLAM frame
    _nf->_T_w_f = frame_ready->getFrame2WorldTransform();

    // Set the current frame as the reference frame
    setRef(_nf->_gnss_meas->llh_meas);
    Eigen::Affine3d T_n_f = Eigen::Affine3d::Identity();
    _nf->_T_n_f           = T_n_f;

    // add absolute pose contraint
    AbsolutePositionFactor af;
    af.t   = _nf->_T_n_f.translation();
    af.nf  = _nf;
    af.inf = Eigen::Matrix3d::Identity();
    af.inf << std::sqrt(1 / _nf->_gnss_meas->cov(0)), 0, 0, 0, std::sqrt(1 / _nf->_gnss_meas->cov(1)),
        0, 0, 0, std::sqrt(1 / _nf->_gnss_meas->cov(2));
    af.inf *= 0.001;
    _pg->_nf_absfact_map.emplace(_nf, af);

    // Add to the nav frame vector
    _nav_frames.push_back(_nf);

    // Calibrate the orientation

    // Add frames until a reasonable displacement is performed
    while (_nf->_T_n_f.translation().norm() < 5) {
        step();
    }

    // Then compute the yaw between ENU and W
    // and update the poses
    calibrateRotation();

    _is_init = true;
}

void Pipeline::step() {
    // Get the last frame in the queue
    _nf = next();
    _slam->_slam_param->getDataProvider()->addFrameToTheQueue(_nf->_frame);

    // Wait for a frame with GPS
    std::shared_ptr<isae::Frame> frame_ready;
    while (_nf->_gnss_meas == nullptr) {
        _nf = next();
        _slam->_slam_param->getDataProvider()->addFrameToTheQueue(_nf->_frame);

        // Ignore if IMU only
        if (_nf->_frame->getSensors().size() == 0)
            continue;

        // Get the ouput from the SLAM
        std::shared_ptr<isae::Frame> frame_ready = _slam->_frame_to_display;
        while (frame_ready != _nf->_frame) {
            frame_ready = _slam->_frame_to_display;
        }

        _nf->_T_w_f = _T_n_w * frame_ready->getFrame2WorldTransform();

        // Compute the current pose
        Eigen::Affine3d T_n_flast = _nav_frames.back()->_T_n_f;
        Eigen::Affine3d T_flast_f = _nav_frames.back()->_T_w_f.inverse() * _nf->_T_w_f;
        _T_n_f                    = T_n_flast * T_flast_f;
    }

    // Compute position in the local frame
    Eigen::Vector3d t_n_f            = ecefToENU(llhToEcef(_nf->_gnss_meas->llh_meas));
    Eigen::Affine3d T_n_f            = Eigen::Affine3d::Identity();
    T_n_f.translation()              = t_n_f;
    T_n_f.affine().block(0, 0, 3, 3) = _nf->_T_w_f.rotation();

    // Set pose
    _nf->_T_n_f = T_n_f;

    // add absolute pose contraint
    AbsolutePositionFactor af;
    af.t   = _nf->_T_n_f.translation();
    af.nf  = _nf;
    af.inf = Eigen::Matrix3d::Identity();
    af.inf << std::sqrt(1 / _nf->_gnss_meas->cov(0)), 0, 0, 0, std::sqrt(1 / _nf->_gnss_meas->cov(1)),
        0, 0, 0, std::sqrt(1 / _nf->_gnss_meas->cov(2));
    af.inf *= 0.001;
    _pg->_nf_absfact_map.emplace(_nf, af);

    // add relative pose constraints
    RelativePoseFactor rf;
    rf.nf_a                  = _nav_frames.back();
    rf.nf_b                  = _nf;
    rf.T_a_b                 = _nav_frames.back()->_T_w_f.inverse() * _nf->_T_w_f;
    rf.inf                   = Eigen::MatrixXd::Identity(6, 6);
    rf.inf.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity(); // cm accuracy
    _pg->_nf_relfact_map.emplace(_nf, rf);

    // Add to the nav frame vector
    _nav_frames.push_back(_nf);

    // Solve pg
    if (_is_init) {
        _pg->solveGraph();
    }
}

void Pipeline::run() {

    while (true) {

        if (!_is_init)
            init();
        else
            step();
    }
}

void Pipeline::calibrateRotation() {

    // Build the ceres problem
    ceres::Problem problem;
    ceres::LossFunction *loss_function = nullptr;

    double theta[1] = {0.0};
    problem.AddParameterBlock(theta, 1);

    // Add all constraints
    for (auto &nf : _nav_frames) {
        ceres::CostFunction *cost_fct =
            new OrientationCalib2D(nf->_T_w_f.translation(), nf->_T_n_f.translation(), Eigen::Matrix2d::Identity());

        problem.AddResidualBlock(cost_fct, loss_function, theta);
    }

    // Solve the problem we just built
    ceres::Solver::Options options;
    options.trust_region_strategy_type         = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type                 = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations                 = 40;
    options.minimizer_progress_to_stdout       = false;
    options.use_explicit_schur_complement      = true;
    options.function_tolerance                 = 1e-3;
    options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.num_threads                        = 4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    // Update the parameter and the poses
    _T_n_w = Eigen::Affine3d::Identity();
    _T_n_w.affine().block(0, 0, 3, 3) << std::cos(theta[0]), -std::sin(theta[0]), 0, std::sin(theta[0]),
        std::cos(theta[0]), 0, 0, 0, 1;

    for (auto nf : _nav_frames) {
        nf->_T_w_f                            = _T_n_w * nf->_T_w_f;
        nf->_T_n_f.affine().block(0, 0, 3, 3) = nf->_T_w_f.rotation();
    }
}