#ifndef PIPELINE_H
#define PIPELINE_H

#include <Eigen/Dense>
#include "navframe.hpp"

class Pipeline {
  public:
    Pipeline(){};

    std::vector<std::shared_ptr<NavFrame>> _nav_frames;
};

#endif // PIPELINE_H