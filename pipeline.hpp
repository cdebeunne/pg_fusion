#ifndef PIPELINE_H
#define PIPELINE_H

#include <Eigen/Dense>
#include <poseGraph.hpp>

class Pipeline {
  public:
    Pipeline(){};

    std::shared_ptr<PoseGraph> pg;
};

#endif // PIPELINE_H