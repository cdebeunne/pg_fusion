#ifndef MOTIONMODEL_HPP
#define MOTIONMODEL_HPP

#include "poseGraph.hpp"
#include <Eigen/Dense>

class motionModel
{
public:
    motionModel(/* args */);
    ~motionModel();

    RelativePoseFactor computeRelativePoseFactor();
    
    Eigen::Vector3d predPosition(Eigen::Vector3d x0, Eigen::Vector3d x1, double dt);
    Eigen::Vector3d predPosition(Eigen::Vector3d x, Eigen::Vector3d v, double dt);


private:
    /* data */
};

motionModel::motionModel(/* args */)
{
}

motionModel::~motionModel()
{
}

#endif