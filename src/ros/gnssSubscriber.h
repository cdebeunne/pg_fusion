#ifndef GNSSSUBSCRIBER_H
#define GNSSSUBSCRIBER_H

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
        std::cout << "[PG] Listening to: " << _gnss_topic << std::endl;
    }

    void subUbx(const sensor_msgs::msg::NavSatFix &gnss_msg) {
        std::lock_guard<std::mutex> lock(_gnss_mutex);    
        // Extract ts from msg
        rclcpp::Time ts            = gnss_msg.header.stamp;
        unsigned long long ts_long = (unsigned long long)ts.nanoseconds();
        std::stringstream  msg;
        msg << "[GNSS SUB] GNSS meas. received. Timestamp: " << ts_long << std::endl;
        msg << "[GNSS SUB] Current time: " <<  this->now().nanoseconds() << std::endl;

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
        if (_gnss_buf.size() > __max_buf_len) {
            _gnss_buf.pop();
            msg << "[GNSS SUB] Warning: GNSS buffer full! Discarded measurement." << std::endl;
        }
        _gnss_buf.push(gnss_meas);

        msg << "[GNSS SUB] GNSS Buffer contains " << _gnss_buf.size() << " elements." << std::endl;
        std::cout << msg.str();
    }

    void pop() {
        std::lock_guard<std::mutex> lock(_gnss_mutex);      
        if (!_gnss_buf.empty()) {
            _gnss_buf.pop();
        }
    }

    bool empty() {        
        std::lock_guard<std::mutex> lock(_gnss_mutex);
        return _gnss_buf.empty();
    }
    
    std::size_t size() {
        std::lock_guard<std::mutex> lock(_gnss_mutex);        
        return _gnss_buf.size();
    } 

    /**
     * Report the timestamp of the next available GNSS measurement
     * (oldest element if multiple measurements are in the queue)
     */
    unsigned long long getTimeStamp() {
        std::lock_guard<std::mutex> lock(_gnss_mutex);
        unsigned long long t = 0;
        if (!_gnss_buf.empty()) {
            t = _gnss_buf.front()->ts_long;
        }
        return t;
    }

    /**
     * Retrieve the next GNSS measurement and remove it from the queue.
     * Returned element is NULL if queue is empty.
     */
    std::shared_ptr<GNSSMeas> getMeas() {
        std::lock_guard<std::mutex> lock(_gnss_mutex);
        std::shared_ptr<GNSSMeas> meas = nullptr;
        if (!_gnss_buf.empty()) {
            meas = _gnss_buf.front();
            _gnss_buf.pop();
        }
        return meas;
    }

    void sync_process() {
        std::cout << "\n[PG] Starting the GNSS reader thread!\n";

        std::vector<std::shared_ptr<isae::ASensor>> sensors;        
        rcl_time_point_value_t t_last         = 0;
        rcl_time_point_value_t t_curr         = 0;

        while (true) {
            if (!_gnss_buf.empty()) {
                std::cout << "[GNSS SUB] " << "Found GNSS " << " (" << _gnss_buf.size() << " in queue)"  << std::endl;
                // _gnss_buf.pop();
                
                t_curr = this->now().nanoseconds();
                std::cout << "[GNSS SUB] Current time: " <<  t_curr << std::endl;
                std::cout << "[GNSS SUB] " << (t_curr - t_last)*1e-9 << " s elapsed since last GNSS measurement" << std::endl;
                t_last = t_curr;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cout << "\n[GNSS SUB] GNSS reader SyncProcess thread is terminating!\n";
    }

    void reportSize() {        
        std::stringstream  msg;
        msg << "[PGSS] GNSS queue contains: " << this->size() << " elements" << std::endl;
        std::cout << msg.str();
    }

  protected:
    std::string _gnss_topic;

    std::queue<std::shared_ptr<GNSSMeas>> _gnss_buf;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _subscription_gnss;

    std::mutex _gnss_mutex;
    
    const uint __max_buf_len = 100;
};

#endif