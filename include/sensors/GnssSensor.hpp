#ifndef GNSSSENSOR_HPP
#define GNSSSENSOR_HPP

// NOT USED FOR NOW

#include "isaeslam/ASensor.h"

struct gnss_config : sensor_config {

};

class GnssSensor : public ASensor, public std::enable_shared_from_this<GnssSensor> {
  public:
    GnssSensor() : ASensor("gnss") {}
    ~GnssSensor() {}
  protected:
  private:
}

#endif