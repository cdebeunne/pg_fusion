#ifndef NAVFRAME_H
#define NAVFRAME_H

#include "isaeslam/data/frame.h"
#include <Eigen/Dense>
#include <memory>

struct GNSSMeas {
    Eigen::Vector3d llh_meas;
    Eigen::Vector3d cov;
    int status;
    int service;
};

class NavFrame {
  public:
    NavFrame(){};
    NavFrame(std::shared_ptr<GNSSMeas> gnss_meas, unsigned long long timestamp)
        : _gnss_meas(gnss_meas), _timestamp(timestamp){};
    NavFrame(std::shared_ptr<isae::Frame> frame) : _frame(frame) {
        _timestamp = _frame->getTimestamp();
        _gnss_meas = nullptr;
    };
    NavFrame(std::shared_ptr<isae::Frame> frame, std::shared_ptr<GNSSMeas> gnss_meas)
        : _frame(frame), _gnss_meas(gnss_meas) {
        _timestamp = _frame->getTimestamp();
    };

    std::shared_ptr<isae::Frame> _frame;
    std::shared_ptr<GNSSMeas> _gnss_meas;
    Eigen::Affine3d _T_n_f;
    unsigned long long _timestamp;
};

#endif // NAVFRAME_H
