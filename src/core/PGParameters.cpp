#include "PGParameters.hpp"
#include <iostream>
#include <filesystem>

PGParameters::PGParameters(const std::string config_folder_path) {
    std::cout << "------------------------------------" << std::endl;   
    std::cout << "Loading YAML: " << config_folder_path << std::endl;
    if (!std::filesystem::exists(config_folder_path))
        std::cerr << config_folder_path << " not found!" << std::endl;
    readConfigFile(config_folder_path);
}

void PGParameters::readConfigFile(const std::string &config_folder_path) {
    YAML::Node yaml_file = YAML::LoadFile(config_folder_path);

    _pipe.slam_config_path          = yaml_file["slam_config_path"].as<std::string>();
    _pipe.window_size               = yaml_file["window_size"].as<uint>();
    _pipe.alignment_displacement    = yaml_file["alignment_displacement"].as<float>();

    _gnss.gnss_topic            = yaml_file["gnss_topic"].as<std::string>();
    _gnss.thresh_cov            = yaml_file["thresh_cov"].as<float>();
    _gnss.remove_z_estimate     = yaml_file["remove_z_estimate"].as<bool>();

    std::vector<double> data_T(16);
    data_T                = yaml_file["T_a_f"].as<std::vector<double>>();
    Eigen::Matrix4d M_a_f = Eigen::Map<Eigen::Affine3d::MatrixType>(&data_T[0], 4, 4).transpose();
    _gnss.T_a_f = Eigen::Affine3d(M_a_f);
}