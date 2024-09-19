#include "isaeslam/slamParameters.h"
#include "sensorSubscriber.h"
#include "rosVisualizer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create the SLAM parameter object
  std::string path = "/home/cesar/ros2_ws/src/SaDVIO/ros/config";
  isae::SLAMParameters slam_param(path);

  // Create pipeline
  std::shared_ptr<Pipeline> pipe = std::make_shared<Pipeline>();

  // Start the sensor subscriber
  std::shared_ptr<SensorSubscriber> sensor_subscriber =
      std::make_shared<SensorSubscriber>(slam_param.getDataProvider(), pipe);

  // Launch full odom thread
  std::thread pg_thread(&Pipeline::run, pipe);
  pg_thread.detach();

  // Launch visualizer thread
  std::shared_ptr<RosVisualizer> rv = std::make_shared<RosVisualizer>(); 
  std::thread rv_thread(&RosVisualizer::runVisualizer, rv, pipe);
  rv_thread.detach();

  rclcpp::spin(sensor_subscriber);

  return 0;
}
