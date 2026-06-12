#include "isaeslam/slamParameters.h"
#include "rosVisualizer.hpp"
#include "sensorSubscriber.h"
#include "gnssSubscriber.h"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::cout << "[PG-main] Launching PG!" << std::endl;
    for (uint t = 10; t>0; t--) {
        std::cout << "[PG-main] Wait " << t << " s" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // load config file
    std::string yaml_path = ament_index_cpp::get_package_share_directory("pg_fusion") + "/config.yaml";
    std::cout << "Loading YAML: " << yaml_path << std::endl;
    if (!std::filesystem::exists(yaml_path))
        std::cout << yaml_path << " not found!" << std::endl;
    YAML::Node config = YAML::LoadFile(yaml_path);

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

    // Load Pipeline parameters
    std::vector<double> data_T(16);
    data_T                = config["T_a_f"].as<std::vector<double>>();
    Eigen::Matrix4d M_a_f = Eigen::Map<Eigen::Affine3d::MatrixType>(&data_T[0], 4, 4).transpose();
    Eigen::Affine3d T_a_f(M_a_f);
    double thresh_cov      = config["thresh_cov"].as<double>();
    uint window_size       = config["window_size"].as<uint>();
    bool remove_z_estimate = config["remove_z_estimate"].as<bool>();

    // Create pipeline
    std::shared_ptr<Pipeline> pipe =
        std::make_shared<Pipeline>(SLAM, T_a_f, thresh_cov, window_size, remove_z_estimate);


    // launch a pure GNSS subscriber
    std::shared_ptr<GnssSubscriber> gnss_subscriber =
        std::make_shared<GnssSubscriber>(config["gnss_topic"].as<std::string>());

    // launch a pure Camera subscriber
    std::shared_ptr<CameraSubscriber> cam_subscriber =
        std::make_shared<CameraSubscriber>(slam_param->getDataProvider());

    // Start the sensor subscriber
    std::shared_ptr<SensorSubscriber> sensor_subscriber =
        std::make_shared<SensorSubscriber>(slam_param->getDataProvider(), pipe, cam_subscriber, gnss_subscriber);

    // Launch SLAM thread
    std::thread odom_thread(&isae::SLAMCore::runFullOdom, SLAM);
    odom_thread.detach();

    // Launch pipeline thread
    std::thread pg_thread(&Pipeline::run, pipe);
    pg_thread.detach();

    // Launch visualizer thread
    std::shared_ptr<RosVisualizer> rv = std::make_shared<RosVisualizer>();
    std::thread rv_thread(&RosVisualizer::runVisualizer, rv, pipe);
    rv_thread.detach();

    // Start a thread for providing new measurements to the SLAM
    std::thread cam_thread(&CameraSubscriber::sync_process, cam_subscriber);
    std::thread gnss_thread(&GnssSubscriber::sync_process, gnss_subscriber);
    std::thread sync_thread(&SensorSubscriber::sync_process, sensor_subscriber);

    rclcpp::executors::MultiThreadedExecutor mt_executor;
    mt_executor.add_node(sensor_subscriber);
    mt_executor.add_node(gnss_subscriber);
    mt_executor.add_node(cam_subscriber);
    mt_executor.spin();

    // rclcpp::spin(sensor_subscriber);

    return 0;
}
