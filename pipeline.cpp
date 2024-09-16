#include "pipeline.hpp"

const Eigen::Vector3d Pipeline::llhToEcef(const Eigen::Vector3d &llh)
{
    double Sp = std::sin(radian(llh.x));
    double Cp = cos(radian(llh.y));
    double Sl = sin(radian(llh.y));
    double Cl = cos(radian(llh.y));
    double N = _a / sqrt(1 - _e2 * Sp * Sp);
    Eigen::Vector3d out;
    out.x = (N + llh.z) * Cp * Cl;
    out.y = (N + llh.z) * Cp * Sl;
    out.z = (N * (1 - _e2) + llh.z) * Sp;
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
    _R_n_e[0][0] = -sin(radian(_llh_ref.y));
    _R_n_e[0][1] = +cos(radian(_llh_ref.y));
    _R_n_e[0][2] = 0.0f;

    _R_n_e[1][0] = -sin(radian(_llh_ref.x))*cos(radian(_llh_ref.y));
    _R_n_e[1][1] = -sin(radian(_llh_ref.x))*sin(radian(_llh_ref.y));
    _R_n_e[1][2] = cos(radian(_llh_ref.x));

    _R_n_e[2][0] = cos(radian(_llh_ref.x))*cos(radian(_llh_ref.y));
    _R_n_e[2][1] = cos(radian(_llh_ref.x))*sin(radian(_llh_ref.y));
    _R_n_e[2][2] = sin(radian(_llh_ref.x));
}

