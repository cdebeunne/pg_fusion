#ifndef PIPELINE_H
#define PIPELINE_H

#include "navframe.hpp"
#include "poseGraph.hpp"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <queue>
#include <thread>

double deg2rad = M_PI / 180;

class Pipeline {
  public:
    Pipeline(std::shared_ptr<isae::SLAMCore> slam, Eigen::Affine3d &T_a_f, double thresh_cov)
        : _slam(slam), _T_a_f(T_a_f), _thresh_cov(thresh_cov) {

        // Ellipsoid parameters of the WGS84 convention
        _a  = 6378137.0f;
        _f  = (1.0f / 298.257223563);
        _e2 = 1 - (1 - _f) * (1 - _f);

        _T_n_f   = Eigen::Affine3d::Identity();
        _T_n_w   = Eigen::Affine3d::Identity();
        _is_init = false;
        _pg      = std::make_shared<PoseGraph>();
    };

    void setRef(const Eigen::Vector3d &llh_ref);
    const Eigen::Vector3d llhToEcef(const Eigen::Vector3d &llh);
    const Eigen::Vector3d ecefToENU(const Eigen::Vector3d &ecef);

    std::shared_ptr<NavFrame> next();
    void calibrateRotation();
    void run();
    void init();
    void step();

    std::shared_ptr<isae::SLAMCore> _slam; // VSLAM
    std::shared_ptr<PoseGraph> _pg;        // Pose graph
    bool _is_init;
    double _a, _f, _e2;     // Ellipsoid parameters for Earth coordinates
    Eigen::Affine3d _T_n_f; // Current pose in local ENU frame
    Eigen::Matrix3d _R_n_e; // Rotation between ENU and ECEF
    Eigen::Affine3d _T_n_w; // Rotation between ENU frame and SLAM (world) frame
    Eigen::Affine3d _T_a_f; // Transformation between antena and frame
    double _thresh_cov;     // Threshold on the covariance of GNSS estimates
    Eigen::Vector3d _llh_ref, _ecef_ref; 
    std::queue<std::shared_ptr<NavFrame>> _nf_queue;
    std::vector<std::shared_ptr<NavFrame>> _nav_frames;
    std::shared_ptr<NavFrame> _nf;
};

#endif // PIPELINE_H