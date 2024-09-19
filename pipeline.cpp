#include "pipeline.hpp"

const Eigen::Vector3d Pipeline::llhToEcef(const Eigen::Vector3d &llh)
{
    double Sp = std::sin(llh.x() * deg2rad);
    double Cp = std::cos(llh.y() * deg2rad);
    double Sl = std::sin(llh.y() * deg2rad);
    double Cl = std::cos(llh.y() * deg2rad);
    double N = _a / sqrt(1 - _e2 * Sp * Sp);
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
    _llh_ref = llh_ref;
    _ecef_ref = llhToEcef(_llh_ref);

    // Compute projection matrix PM_ used to project coordinates in LTP
    _R_n_e(0,0) = -std::sin(_llh_ref.y() * deg2rad);
    _R_n_e(0,1) = +std::cos(_llh_ref.y() * deg2rad);
    _R_n_e(0,2) = 0.0f;

    _R_n_e(1,0) = -std::sin(_llh_ref.x() * deg2rad)*std::cos(_llh_ref.y() * deg2rad);
    _R_n_e(1,1) = -std::sin(_llh_ref.x() * deg2rad)*std::sin(_llh_ref.y() * deg2rad);
    _R_n_e(1,2) = std::cos(_llh_ref.x() * deg2rad);

    _R_n_e(2,0) = std::cos(_llh_ref.x() * deg2rad)*std::cos(_llh_ref.y() * deg2rad);
    _R_n_e(2,1) = std::cos(_llh_ref.x() * deg2rad)*std::sin(_llh_ref.y() * deg2rad);
    _R_n_e(2,2) = std::sin(_llh_ref.x() * deg2rad);
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

    // Get the last frame in the queue
    _nf = next();

    // Set the current frame as the reference frame
    setRef(_nf->_llh_meas);
    Eigen::Affine3d T_n_f = Eigen::Affine3d::Identity();
    _nf->_T_n_f = T_n_f;

    // Add to the nav frame vector
    _nav_frames.push_back(_nf);

}

void Pipeline::step() {

    // Get the last frame in the queue
    _nf = next();

    // Compute position in the local frame
    Eigen::Vector3d t_n_f = ecefToENU(llhToEcef(_nf->_llh_meas));
    Eigen::Affine3d T_n_f = Eigen::Affine3d::Identity();
    T_n_f.translation() = t_n_f;

    // Set pose
    _nf->_T_n_f = T_n_f;

    // Add to the nav frame vector
    _nav_frames.push_back(_nf);
}

void Pipeline::run() {

    while (true) {

        if (_nav_frames.size() == 0) 
            init();
        else
            step();

    }
}