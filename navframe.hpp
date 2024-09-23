#ifndef NAVFRAME_H
#define NAVFRAME_H

#include <Eigen/Dense>

struct GNSSMeas {
  Eigen::Vector3d llh_meas;
  Eigen::Vector3d cov;
  int status;
  int service;
};

class NavFrame {
public:
  NavFrame(){};
  NavFrame(GNSSMeas gnss_meas, unsigned long long timestamp)
      : _gnss_meas(gnss_meas), _timestamp(timestamp){};

  GNSSMeas _gnss_meas;
  Eigen::Affine3d _T_n_f;
  unsigned long long _timestamp;
};

#endif // NAVFRAME_H
