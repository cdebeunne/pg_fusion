#ifndef GNSSSENSOR_HPP
#define GNSSSENSOR_HPP

// NOT USED FOR NOW

#include "isaeslam/data/sensors/ASensor.h"

struct gnss_config : isae::sensor_config {

};

enum GNSSCustomService {
  GNSSSERVICE_NO_CONTEXT   = 128,
  GNSSSERVICE_HAS_CONTEXT  = 256,
};

enum ContextLabel {
  CONTEXT_NONE    = 0,
  CONTEXT_CANYON  = 1,
  CONTEXT_OPENSKY = 2,
  CONTEXT_TREES   = 3,
  CONTEXT_URBAN   = 4
};

struct GNSSMeas {
    Eigen::Vector3d llh_meas;
    Eigen::Vector3d cov;
    int status;
    int service;
    unsigned long long ts_long;
};

class GnssSensor : public isae::ASensor, public std::enable_shared_from_this<GnssSensor> {
  public:
    ~GnssSensor() {}

    GnssSensor(const std::shared_ptr<GNSSMeas> meas, float thresh_cov) : 
      isae::ASensor("gnss"), _meas(meas), _thresh_cov(thresh_cov) {
      checkIsUsable();
    }

    GnssSensor(const std::shared_ptr<GNSSMeas> meas) : GnssSensor(meas, 0) {}

    GnssSensor() : GnssSensor(nullptr, 0) {}

    bool isUsable() {return _is_usable;}

    std::shared_ptr<GNSSMeas> _meas;
  protected:

    bool checkIsUsable() {
        if (!_meas) {
          _is_usable = false;
        }
        if (_thresh_cov == 0) {
          _is_usable = true; // value was not set
        }
        if (_meas && _meas->cov.norm() < _thresh_cov) {
          _is_usable = true;
        }
        if (_meas->service == GNSSSERVICE_HAS_CONTEXT && _meas->status == CONTEXT_CANYON) {
          _is_usable = false;
        }
        return _is_usable;
    }

    float _thresh_cov = 0;
    bool _is_usable = false;
  private:
};

#endif