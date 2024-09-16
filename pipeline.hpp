#ifndef PIPELINE_H
#define PIPELINE_H

#include <Eigen/Dense>
#include "navframe.hpp"

class Pipeline {
  public:
    Pipeline(){

      // Ellipsoid parameters of the WGS84 convention
      _a = 6378137.0f;
      _f = (1.0f/298.257223563);
      _e2 = 1-(1-_f)*(1-_f);

    };

    void setRef(const Eigen::Vector3d &llh_ref);
    const Eigen::Vector3d llhToEcef(const Eigen::Vector3d &llh);
    const Eigen::Vector3d ecefToENU(const Eigen::Vector3d &ecef);

    double _a, _f, _e2; // Ellipsoid parameters for Earth coordinates
    Eigen::Matrix3d _R_n_e;
    Eigen::Vector3d _llh_ref, _ecef_ref;
    std::vector<std::shared_ptr<NavFrame>> _nav_frames;
};

#endif // PIPELINE_H