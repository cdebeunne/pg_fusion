#ifndef PGPARAMETERS_HPP
#define PGPARAMETERS_HPP


#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

struct GnssParamStruct {
    std::string gnss_topic;
    float thresh_cov;
    bool remove_z_estimate;
    Eigen::Affine3d T_a_f;
};

struct VisionParamStruct {

};

struct PipelineParamStruct {
    std::string slam_config_path;
    uint window_size;
};

/*!
 * @brief A class that gathers most of the algorithmic blocks of the SLAM system that can be setup in the config file
 *
 * Some attributes are sets as unordered map because these depends on the feature type (e.g. matcher, detector...). Then
 * the proper blocks can be called using the feature label.
 */
class PGParameters {
  public:
    PGParameters(const std::string config_folder_path);

    void readConfigFile(const std::string &config_folder_path);

    PipelineParamStruct _pipe;
    GnssParamStruct _gnss;
    VisionParamStruct _vision;

};

#endif