#include "isaeslam/slamParameters.h"
#include "PGParameters.hpp"
#include "rosVisualizer.hpp"
#include "sensorSynchronizer.h"
#include "gnssSubscriber.h"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::cout << "[PG-main] Launching PG!" << std::endl;
    // for (uint t = 10; t>0; t--) {
    //     std::cout << "[PG-main] Wait " << t << " s" << std::endl;
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    // load config file
    std::string yaml_path = ament_index_cpp::get_package_share_directory("pg_fusion") + "/config.yaml";

    std::shared_ptr<PGParameters> pg_param = std::make_shared<PGParameters>(yaml_path);

    // Create the SLAM parameter object
    std::shared_ptr<isae::SLAMParameters> slam_param = std::make_shared<isae::SLAMParameters>(pg_param->_pipe.slam_config_path);

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
    std::shared_ptr<Pipeline> pipe =
        std::make_shared<Pipeline>(SLAM, pg_param);


    // launch a pure GNSS subscriber
    std::shared_ptr<GnssSubscriber> gnss_subscriber =
        std::make_shared<GnssSubscriber>(pg_param->_gnss.gnss_topic);

    // launch a pure Camera subscriber
    std::shared_ptr<CameraSubscriber> cam_subscriber =
        std::make_shared<CameraSubscriber>(slam_param->getDataProvider());

    // Start the sensor subscriber
    std::shared_ptr<SensorSynchronizer> sensor_subscriber =
        std::make_shared<SensorSynchronizer>(slam_param->getDataProvider(), pipe, cam_subscriber, gnss_subscriber);

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
    // std::thread cam_thread(&CameraSubscriber::sync_process, cam_subscriber);
    // std::thread gnss_thread(&GnssSubscriber::sync_process, gnss_subscriber);
    std::thread sync_thread(&SensorSynchronizer::sync_process, sensor_subscriber);
    sync_thread.detach();

    rclcpp::executors::MultiThreadedExecutor mt_executor;
    mt_executor.add_node(sensor_subscriber);
    mt_executor.add_node(gnss_subscriber);
    mt_executor.add_node(cam_subscriber);
    mt_executor.spin();

    // rclcpp::spin(sensor_subscriber);

    return 0;
}
