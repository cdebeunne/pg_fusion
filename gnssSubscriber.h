#include "isaeslam/data/frame.h"
#include "isaeslam/dataproviders/adataprovider.h"

#include <mutex>
#include <thread>

#include "pipeline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GnssSubscriber : public rclcpp::Node {

  public:
    GnssSubscriber(std::string gnss_topic)
        : Node("gnss_subscriber"), _gnss_topic(gnss_topic) {
        _subscription_gnss = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gnss_topic, 10, std::bind(&GnssSubscriber::subUbx, this, std::placeholders::_1));
    }

    void subUbx(const sensor_msgs::msg::NavSatFix &gnss_msg) {
        // Extract ts from msg
        rclcpp::Time ts            = gnss_msg.header.stamp;
        unsigned long long ts_long = (unsigned long long)ts.nanoseconds();
        std::cout << "[GS] GNSS meas. received. Timestamp: " << ts_long << std::endl;
        std::cout << "[GS] Current time: " <<  this->now().nanoseconds() << std::endl;

        // Build gnss measurement
        std::shared_ptr<GNSSMeas> gnss_meas = std::make_shared<GNSSMeas>();
        gnss_meas->llh_meas =
            Eigen::Vector3d((double)gnss_msg.latitude, (double)gnss_msg.longitude, (double)gnss_msg.altitude);
        gnss_meas->cov     = Eigen::Vector3d((double)gnss_msg.position_covariance[0],
                                         (double)gnss_msg.position_covariance[4],
                                         (double)gnss_msg.position_covariance[8]);
        gnss_meas->status  = gnss_msg.status.status;
        gnss_meas->service = gnss_msg.status.service;
        gnss_meas->ts_long = ts_long;

        // push the message in the buffer
        _gnss_buf.push(gnss_meas);
        std::cout << "[GS] GNSS Buffer contains " << _gnss_buf.size() << " elements." << std::endl;
    }

    void sync_process() {
        std::cout << "\nStarting the GNSS reader thread!\n";

        std::vector<std::shared_ptr<isae::ASensor>> sensors;        
        rcl_time_point_value_t t_last         = 0;
        rcl_time_point_value_t t_curr         = 0;

        while (true) {
            if (!_gnss_buf.empty()) {
                std::cout << "Found GNSS " << " (" << _gnss_buf.size() << " in queue)"  << std::endl;
                _gnss_buf.pop();
                
                t_curr = this->now().nanoseconds();
                std::cout << (t_curr - t_last)*1e-9 << " s elapsed since last GNSS measurement" << std::endl;
                t_last = t_curr;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cout << "\n GNSS reader SyncProcess thread is terminating!\n";
    }

    std::string _gnss_topic;

    std::queue<std::shared_ptr<GNSSMeas>> _gnss_buf;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _subscription_gnss;
};
