#ifndef PIPELINE_H
#define PIPELINE_H

#include <Eigen/Dense>
#include "navframe.hpp"
#include <ceres/ceres.h>
#include "poseGraph.hpp"
#include <queue>
#include <memory>
#include <iostream>
#include <cmath>
#include <thread>

double deg2rad = M_PI / 180;

class Pipeline {
  public:
    Pipeline(std::shared_ptr<isae::SLAMCore> slam): _slam(slam){

      // Ellipsoid parameters of the WGS84 convention
      _a = 6378137.0f;
      _f = (1.0f/298.257223563);
      _e2 = 1-(1-_f)*(1-_f);

      _T_n_w = Eigen::Affine3d::Identity();
      _is_init = false;
      _pg = std::make_shared<PoseGraph>();

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
    std::shared_ptr<PoseGraph> _pg; // Pose graph
    bool _is_init;
    double _a, _f, _e2; // Ellipsoid parameters for Earth coordinates
    Eigen::Matrix3d _R_n_e;
    Eigen::Affine3d _T_n_w;
    Eigen::Vector3d _llh_ref, _ecef_ref;
    std::queue<std::shared_ptr<NavFrame>> _nf_queue;
    std::vector<std::shared_ptr<NavFrame>> _nav_frames;
    std::shared_ptr<NavFrame> _nf;
};

#endif // PIPELINE_H