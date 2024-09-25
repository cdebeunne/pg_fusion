#include "isaeslam/data/frame.h"
#include "isaeslam/dataproviders/adataprovider.h"

#include <mutex>
#include <thread>

#include "pipeline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_msgs/msg/nav_pvt.hpp"
#include <cv_bridge/cv_bridge.h>

class SensorSubscriber : public rclcpp::Node {

  public:
    SensorSubscriber(std::shared_ptr<isae::ADataProvider> prov, std::shared_ptr<Pipeline> pipe, std::string gnss_topic)
        : Node("sensor_subscriber"), _prov(prov), _pipe(pipe), _gnss_topic(gnss_topic) {

        _cam_l_topic       = _prov->getCamConfigs().at(0)->ros_topic;
        _subscription_left = this->create_subscription<sensor_msgs::msg::Image>(
            _cam_l_topic, 10, std::bind(&SensorSubscriber::subLeftImage, this, std::placeholders::_1));
        _subscription_gnss = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gnss_topic, 10, std::bind(&SensorSubscriber::subUbx, this, std::placeholders::_1));
        if (_prov->getNCam() == 2) {
            _cam_r_topic        = prov->getCamConfigs().at(1)->ros_topic;
            _subscription_right = this->create_subscription<sensor_msgs::msg::Image>(
                _cam_r_topic, 10, std::bind(&SensorSubscriber::subRightImage, this, std::placeholders::_1));
        }
        if (_prov->getIMUConfig()) {
            _imu_topic        = _prov->getIMUConfig()->ros_topic;
            _subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
                _imu_topic, 10, std::bind(&SensorSubscriber::subIMU, this, std::placeholders::_1));
        }
    }

    void subLeftImage(const sensor_msgs::msg::Image &img_msg) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        _imgs_bufl.push(img_msg);
    }

    void subUbx(const sensor_msgs::msg::NavSatFix &gnss_msg) {
        // Extract ts from msg
        rclcpp::Time ts            = gnss_msg.header.stamp;
        unsigned long long ts_long = (unsigned long long)ts.nanoseconds();

        // Build gnns measurement
        std::shared_ptr<GNSSMeas> gnss_meas = std::make_shared<GNSSMeas>();
        gnss_meas->llh_meas =
            Eigen::Vector3d((double)gnss_msg.latitude, (double)gnss_msg.longitude, (double)gnss_msg.altitude);
        gnss_meas->cov     = Eigen::Vector3d((double)gnss_msg.position_covariance[0],
                                         (double)gnss_msg.position_covariance[4],
                                         (double)gnss_msg.position_covariance[8]);
        gnss_meas->status  = gnss_msg.status.status;
        gnss_meas->service = gnss_msg.status.service;

        // push the message in the buffer
        _gnss_buf.push(gnss_meas);
    }

    void subRightImage(const sensor_msgs::msg::Image &img_msg) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        _imgs_bufr.push(img_msg);
    }

    void subIMU(const sensor_msgs::msg::Imu &imu_msg) {
        std::lock_guard<std::mutex> lock(_imu_mutex);
        _imu_buf.push(imu_msg);
    }

    cv::Mat getGrayImageFromMsg(const sensor_msgs::msg::Image &img_msg) {
        // Get and prepare images
        cv_bridge::CvImagePtr ptr;
        try {
            ptr = cv_bridge::toCvCopy(img_msg, "mono8");
        } catch (cv_bridge::Exception &e) {
            std::cout << "\n\n\ncv_bridge exeception: %s\n\n\n" << e.what() << std::endl;
        }

        return ptr->image;
    }

    void getImuInfoFromMsg(const sensor_msgs::msg::Imu &imu_msg, Eigen::Vector3d &acc, Eigen::Vector3d &gyr) {

        // Extract the acceleration and gyroscope values from the IMU message
        double ax = imu_msg.linear_acceleration.x;
        double ay = imu_msg.linear_acceleration.y;
        double az = imu_msg.linear_acceleration.z;
        double gx = imu_msg.angular_velocity.x;
        double gy = imu_msg.angular_velocity.y;
        double gz = imu_msg.angular_velocity.z;

        // Create an Eigen vector for the acceleration and gyroscope values
        acc << ax, ay, az;
        gyr << gx, gy, gz;
    }

    void sync_process() {
        std::cout << "\nStarting the measurements reader thread!\n";

        std::vector<std::shared_ptr<isae::ASensor>> sensors;
        double time_tolerance = 0.0025; // TODO add this as a parameter of the yaml
        double t_last         = 0;
        double t_curr         = 0;

        while (true) {

            // Image messages
            cv::Mat image0, image1;
            std::vector<cv::Mat> imgs;
            std::vector<std::shared_ptr<isae::ImageSensor>> img_sensors;

            // Case Stereo
            if (_prov->getNCam() == 2) {
                if (!_imgs_bufl.empty() && !_imgs_bufr.empty()) {
                    double time0 = _imgs_bufl.front().header.stamp.sec * 1e9 + _imgs_bufl.front().header.stamp.nanosec;
                    double time1 = _imgs_bufr.front().header.stamp.sec * 1e9 + _imgs_bufr.front().header.stamp.nanosec;
                    t_curr       = time0;

                    // sync tolerance
                    if (time0 < time1 - 20000000) {
                        _imgs_bufl.pop();
                        std::cout << "\n Throw img0 -- Sync error : " << (time0 - time1) << "\n";
                    } else if (time0 > time1 + 20000000) {
                        _imgs_bufr.pop();
                        std::cout << "\n Throw img1 -- Sync error : " << (time0 - time1) << "\n";
                    } else {

                        // Check if this measurement can be added to the current frame
                        if (std::abs(t_curr - t_last) * 1e-9 > time_tolerance && !sensors.empty()) {

                            // Create a frame with the stored sensors
                            std::shared_ptr<isae::Frame> f = std::shared_ptr<isae::Frame>(new isae::Frame());
                            f->init(sensors, t_last);

                            // Add a gnss measurement if available and images in the frame
                            if (_gnss_buf.empty() || f->getSensors().empty()) {
                                _pipe->_nf_queue.push(std::make_shared<NavFrame>(f));
                            } else {
                                _pipe->_nf_queue.push(std::make_shared<NavFrame>(f, _gnss_buf.front()));
                                _gnss_buf.pop();
                            }

                            sensors.clear();
                        }

                        _img_mutex.lock();
                        image0 = getGrayImageFromMsg(_imgs_bufl.front());
                        image1 = getGrayImageFromMsg(_imgs_bufr.front());
                        _imgs_bufl.pop();
                        _imgs_bufr.pop();
                        _img_mutex.unlock();

                        imgs.push_back(image0);
                        imgs.push_back(image1);

                        img_sensors = _prov->createImageSensors(imgs);
                        sensors.push_back(img_sensors.at(0));
                        sensors.push_back(img_sensors.at(1));
                    }

                    t_last = t_curr;
                }

                // Case mono
            } else {
                if (!_imgs_bufl.empty()) {
                    std::lock_guard<std::mutex> lock(_img_mutex);
                    t_curr = _imgs_bufl.front().header.stamp.sec * 1e9 + _imgs_bufl.front().header.stamp.nanosec;

                    // Check if this measurement can be added to the current frame
                    if (std::abs(t_curr - t_last) * 1e-9 > time_tolerance && !sensors.empty()) {

                        // Create a frame with the stored sensors
                        std::shared_ptr<isae::Frame> f = std::shared_ptr<isae::Frame>(new isae::Frame());
                        f->init(sensors, t_last);

                        // Add a gnss measurement if available and images in the frame
                        if (_gnss_buf.empty() || f->getSensors().empty()) {
                            _pipe->_nf_queue.push(std::make_shared<NavFrame>(f));
                        } else {
                            _pipe->_nf_queue.push(std::make_shared<NavFrame>(f, _gnss_buf.front()));
                            _gnss_buf.pop();
                        }

                        sensors.clear();
                    }

                    image0 = getGrayImageFromMsg(_imgs_bufl.front());
                    _imgs_bufl.pop();
                    imgs.push_back(image0);

                    img_sensors = _prov->createImageSensors(imgs);
                    sensors.push_back(img_sensors.at(0));

                    t_last = t_curr;
                }
            }

            // IMU message
            if (!_imu_buf.empty()) {
                t_curr = _imu_buf.front().header.stamp.sec * 1e9 + _imu_buf.front().header.stamp.nanosec;

                // Check if this measurement can be added to the current frame
                if (std::abs(t_curr - t_last) * 1e-9 > time_tolerance && !sensors.empty()) {

                    // Create a frame with the stored sensors
                    std::shared_ptr<isae::Frame> f = std::shared_ptr<isae::Frame>(new isae::Frame());
                    f->init(sensors, t_last);

                    // Add a gnss measurement if available
                    if (_gnss_buf.empty() || f->getSensors().empty()) {
                        _pipe->_nf_queue.push(std::make_shared<NavFrame>(f));
                    } else {
                        _pipe->_nf_queue.push(std::make_shared<NavFrame>(f, _gnss_buf.front()));
                        _gnss_buf.pop();
                    }

                    sensors.clear();
                }

                Eigen::Vector3d acc, gyr;
                _imu_mutex.lock();
                getImuInfoFromMsg(_imu_buf.front(), acc, gyr);
                _imu_buf.pop();
                _imu_mutex.unlock();
                std::shared_ptr<isae::IMU> imu_ptr = _prov->createImuSensor(acc, gyr);
                sensors.push_back(imu_ptr);

                t_last = t_curr;
            }
        }

        std::cout << "\n Bag reader SyncProcess thread is terminating!\n";
    }

    std::shared_ptr<isae::ADataProvider> _prov;
    std::shared_ptr<Pipeline> _pipe;
    std::string _cam_l_topic, _cam_r_topic, _imu_topic, _gnss_topic;

    std::queue<sensor_msgs::msg::Image> _imgs_bufl, _imgs_bufr;
    std::queue<sensor_msgs::msg::Imu> _imu_buf;
    std::queue<std::shared_ptr<GNSSMeas>> _gnss_buf;
    std::mutex _img_mutex, _imu_mutex;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription_left;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription_right;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subscription_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _subscription_gnss;
};