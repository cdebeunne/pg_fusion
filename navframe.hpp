#ifndef NAVFRAME_H
#define NAVFRAME_H

#include <Eigen/Dense>

class NavFrame {
  public:
    NavFrame(){};
    NavFrame(Eigen::Vector3d llh_meas, unsigned long long timestamp): _llh_meas(llh_meas), _timestamp(timestamp) {};

    Eigen::Vector3d _llh_meas;
    Eigen::Affine3d _T_n_f;
    unsigned long long _timestamp;
};

#endif // NAVFRAME_H