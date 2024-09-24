#include "isaeslam/slamParameters.h"
#include "rosVisualizer.hpp"
#include "sensorSubscriber.h"
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // load config file
    YAML::Node config = YAML::LoadFile(ament_index_cpp::get_package_share_directory("pg_fusion") + "/config.yaml");

    // Create the SLAM parameter object
    std::string path                                 = config["slam_config_path"].as<std::string>();
    std::shared_ptr<isae::SLAMParameters> slam_param = std::make_shared<isae::SLAMParameters>(path);

    std::shared_ptr<isae::SLAMCore> SLAM;

    if (slam_param->_config.slam_mode == "bimono")
        SLAM = std::make_shared<isae::SLAMBiMono>(slam_param);
    else if (slam_param->_config.slam_mode == "mono")
        SLAM = std::make_shared<isae::SLAMMono>(slam_param);
    else if (slam_param->_config.slam_mode == "nofov")
        SLAM = std::make_shared<isae::SLAMNonOverlappingFov>(slam_param);
    else if (slam_param->_config.slam_mode == "bimonovio")
        SLAM = std::make_shared<isae::SLAMBiMonoVIO>(slam_param);
    else if (slam_param->_config.slam_mode == "monovio")
        SLAM = std::make_shared<isae::SLAMMonoVIO>(slam_param);

    // Create pipeline
    std::shared_ptr<Pipeline> pipe = std::make_shared<Pipeline>(SLAM);

    // Start the sensor subscriber
    std::shared_ptr<SensorSubscriber> sensor_subscriber = std::make_shared<SensorSubscriber>(
        slam_param->getDataProvider(), pipe, config["gnss_topic"].as<std::string>());

    // Launch full odom thread
    std::thread pg_thread(&Pipeline::run, pipe);
    pg_thread.detach();

    // Launch visualizer thread
    std::shared_ptr<RosVisualizer> rv = std::make_shared<RosVisualizer>();
    std::thread rv_thread(&RosVisualizer::runVisualizer, rv, pipe);
    rv_thread.detach();

    // Start a thread for providing new measurements to the SLAM
    std::thread sync_thread(&SensorSubscriber::sync_process, sensor_subscriber);

    rclcpp::spin(sensor_subscriber);

    return 0;
}
