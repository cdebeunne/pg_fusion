#ifndef NAVFRAME_H
#define NAVFRAME_H

#include "isaeslam/data/frame.h"
#include "sensors/GnssSensor.hpp"
#include <Eigen/Dense>
#include <memory>
#include <iostream>

/*! NavFrame
 * @brief A "Navigation Frame", i.e. a pose in an absolute coordinate system, triggered by the arrival of a useful GNSS measurement.
 *
 * A frame is the core component of a SLAM system. It gathers measurements for one or more sensors (e.g., cameras, IMUs)
 * at a specific timestamp. It contains pointers to sensor objects as well as pointer to landmarks that serves for
 * mapping and localization. A frame will be located in space with its pose and eventually, a covariance will
 * be assoiated to it. If a frame is considered as a keyframe, it will be incorporated in a map.
 */
class NavFrame {
  public:
    NavFrame(){};
    NavFrame(std::shared_ptr<GnssSensor> gnss, unsigned long long timestamp)
        : _gnss(gnss), _timestamp(timestamp) {
        _is_aligned = false;
        _T_n_w = Eigen::Affine3d::Identity();
    };
    NavFrame(std::shared_ptr<isae::Frame> frame) : _frame(frame) {
        _timestamp  = _frame->getTimestamp();
        _gnss  = nullptr;
        _is_aligned = false;
        _T_n_w = Eigen::Affine3d::Identity();
    };
    NavFrame(std::shared_ptr<isae::Frame> frame, std::shared_ptr<GnssSensor> gnss)
        : _frame(frame), _gnss(gnss) {
        _timestamp  = _frame->getTimestamp();
        if (_timestamp >= gnss->_meas->ts_long)
        {            
            if ((_timestamp - gnss->_meas->ts_long) > 1e9)
            {
                std::cout << "Offset Frame -> GNSS" << (_timestamp - gnss->_meas->ts_long) << std::endl;
                // throw std::runtime_error("Time offset too large (1)");
            }
        }
        else
        {            
            if ((gnss->_meas->ts_long - _timestamp) > 1e9)
            {
                std::cout << "Offset Frame -> GNSS" << (_timestamp - gnss->_meas->ts_long) << std::endl;
                // throw std::runtime_error("Time offset too large (2)");
            }
        }

        _is_aligned = false;
        _T_n_w = Eigen::Affine3d::Identity();
    };

    // TODO add unique _ID
    // TODO add counter
    // Maybe inherit from Frame?
    std::shared_ptr<isae::Frame> _frame;
    std::shared_ptr<GnssSensor> _gnss;
    Eigen::Affine3d _T_n_f, _T_w_f, _T_n_w;
    bool _is_aligned;
    unsigned long long _timestamp;
};

#endif // NAVFRAME_H
