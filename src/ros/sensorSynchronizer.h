#include "isaeslam/data/frame.h"
#include "isaeslam/dataproviders/adataprovider.h"

#include <mutex>
#include <thread>

#include "gnssSubscriber.h"
#include "cameraSubscriber.h"
#include "pipeline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <cv_bridge/cv_bridge.hpp>

class SensorSynchronizer : public rclcpp::Node {

  public:
    SensorSynchronizer(std::shared_ptr<isae::ADataProvider> prov, std::shared_ptr<Pipeline> pipe, 
    std::shared_ptr<CameraSubscriber> cam_sub, std::shared_ptr<GnssSubscriber> gnss_sub)
        : Node("sensor_subscriber"), _prov(prov), _pipe(pipe), _cam_sub(cam_sub), _gnss_sub(gnss_sub) {
        // if (_prov->getIMUConfig()) {
        //     _imu_topic        = _prov->getIMUConfig()->ros_topic;
        //     _subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        //         _imu_topic, 10, std::bind(&SensorSynchronizer::subIMU, this, std::placeholders::_1));
        // }
    }

    // void subIMU(const sensor_msgs::msg::Imu &imu_msg) {
    //     std::lock_guard<std::mutex> lock(_imu_mutex);
    //     _imu_buf.push(imu_msg);
    // }


    // void getImuInfoFromMsg(const sensor_msgs::msg::Imu &imu_msg, Eigen::Vector3d &acc, Eigen::Vector3d &gyr) {

    //     // Extract the acceleration and gyroscope values from the IMU message
    //     double ax = imu_msg.linear_acceleration.x;
    //     double ay = imu_msg.linear_acceleration.y;
    //     double az = imu_msg.linear_acceleration.z;
    //     double gx = imu_msg.angular_velocity.x;
    //     double gy = imu_msg.angular_velocity.y;
    //     double gz = imu_msg.angular_velocity.z;

    //     // Create an Eigen vector for the acceleration and gyroscope values
    //     acc << ax, ay, az;
    //     gyr << gx, gy, gz;
    // }

    void sync_process() {
        std::cout << "\n[PG] Starting the measurements reader thread!\n";

        std::vector<std::shared_ptr<isae::ASensor>> sensors;
        double time_tolerance = 0.0025; // TODO add this as a parameter of the yaml
        unsigned long long t_last         = 0;
        unsigned long long t_curr         = 0;
        unsigned long long ts_gnss_last = 0;
        unsigned long long ts_gnss_curr = 0;
        rcl_time_point_value_t t_gnss_last = 0;
        rcl_time_point_value_t t_gnss_curr = 0;
        double time_tol_gnss_s = 60;

        while (true) {
            // GNSS message
            // GNSSMeas gnss_fix;

            // Image messages
            cv::Mat image0, image1;
            std::vector<cv::Mat> imgs;
            std::vector<std::shared_ptr<isae::ImageSensor>> img_sensors;

            // Case Stereo
            if (_prov->getNCam() == 2) {
                if (!_cam_sub->emptyAny()) {

                    std::stringstream msg;
                    msg << "[PGSS] Found STEREO image in buffer! " 
                        << "(" << _cam_sub->size(0) << "|" << _cam_sub->size(1) << ")" << std::endl;
                    std::cout << msg.str();

                    // other threads can add to, but 
                    std::vector<rclcpp::Time> t = _cam_sub->getTimeStamps();
                    unsigned long long time0 = t.at(0).nanoseconds();
                    unsigned long long time1 = t.at(1).nanoseconds();
                    t_curr       = time0;

                    std::stringstream().swap(msg);
                    msg << "[PGSS] Frame timestamp: " << t_curr << std::endl;
                    std::cout << msg.str();

                    // sync tolerance
                    if (time0 < time1 - 25000000) {
                        _cam_sub->pop(0);
                        std::cout << "\n Throw img0 -- Sync error : " << (time0 - time1) << "\n";
                    } else if (time0 > time1 + 25000000) {
                        _cam_sub->pop(1);
                        std::cout << "\n Throw img1 -- Sync error : " << (time0 - time1) << "\n";
                    } else {

                        // Check if this measurement can be added to the current frame
                        if (std::abs(t_curr*1e-9 - t_last*1e-9)  > time_tolerance && !sensors.empty()) {

                            // Create a frame with the stored sensors
                            std::shared_ptr<isae::Frame> f = std::shared_ptr<isae::Frame>(new isae::Frame());
                            f->init(sensors, t_last);

                            std::cout << "[PGSS] GNSS queue contains: " << _gnss_sub->size() << " elements" << std::endl;

                            // Add a gnss measurement if available and images in the frame
                            if (_gnss_sub->empty() || f->getSensors().empty()) {
                                _pipe->_nf_queue.push(std::make_shared<NavFrame>(f));

                                std::stringstream().swap(msg);
                                msg << "[PGSS] Created NF (stereo) " << "( " << _pipe->_nf_queue.size() << "in queue)" << std::endl;
                                std::cout << msg.str();
                            } else {
                                
                                // check if the step between two GNSS measurement timestamps 
                                // is similar to the step in wall time
                                // otherwise delayed measurements can ruin the graph
                                ts_gnss_curr = _gnss_sub->getTimeStamp();
                                t_gnss_curr = this->now().nanoseconds();
                                double dt = std::abs((ts_gnss_curr - ts_gnss_last)*1e-9 - (t_gnss_curr - t_gnss_last)*1e-9);
                                if (ts_gnss_last && // initial value is zero
                                     dt > time_tol_gnss_s)
                                {
                                    _gnss_sub->pop();
                                    std::stringstream().swap(msg);
                                    msg << "############################################################" << std::endl;
                                    msg << "[PGSS] Discarded NF (stereo | GNSS) " << " (" << dt << " s)"  << std::endl;
                                    msg << "############################################################" << std::endl;
                                    std::cout << msg.str();
                                } 
                                else
                                {
                                    std::shared_ptr<GNSSMeas> gnss_meas = _gnss_sub->getMeas();
                                    if (gnss_meas)
                                        _pipe->_nf_queue.push(std::make_shared<NavFrame>(f, std::make_shared<GnssSensor>(gnss_meas, _pipe->_param->_gnss.thresh_cov)));
                                    else
                                        std::cerr << "[PGSS] GNSS queue is not empty, but no measurement was obtained!" << std::endl;

                                    std::stringstream().swap(msg);
                                    msg << "[PGSS] Created NF (stereo | GNSS) " << " (" << _pipe->_nf_queue.size() << " in queue)"  << std::endl;
                                    std::cout << msg.str();
                                }
                                ts_gnss_last = ts_gnss_curr;
                                t_gnss_last = t_gnss_curr;
                            }

                            sensors.clear();
                        } else {
                            msg << "[PGSS] Sensors empty!" << std::endl;
                            if (std::abs(t_curr*1e-9 - t_last*1e-9) <= time_tolerance) {
                                std::stringstream().swap(msg);
                                msg << "[PGSS] Time tolerance violated: " << std::abs(t_curr*1e-9 - t_last*1e-9) << " <= " << time_tolerance << std::endl;
                                std::cout << msg.str();
                            }
                            if (sensors.empty()) {
                                std::stringstream().swap(msg);
                                msg << "[PGSS] Sensors empty!" << std::endl;
                                std::cout << msg.str();
                            }
                        }


                        image0 = _cam_sub->getGrayImageMono(0);
                        if (!image0.empty())
                            imgs.push_back(image0);

                        image1 = _cam_sub->getGrayImageMono(1);
                        if (!image1.empty())
                            imgs.push_back(image1);

                        if (!imgs.empty())
                            img_sensors = _prov->createImageSensors(imgs);

                        if (!img_sensors.empty()) {
                            sensors.push_back(img_sensors.at(0));
                            sensors.push_back(img_sensors.at(1));
                        }
                    }

                    t_last = t_curr;
                }

                // Case mono
            } 
            // else 
            // {
            //     if (!_imgs_bufl.empty()) {
            //         std::stringstream msg;
            //         msg << "Found MONO image in buffer! " 
            //             << "(" << _imgs_bufl.size() << ")" << std::endl;
            //         std::cout << msg.str();
            //         std::lock_guard<std::mutex> lock(_img_mutex);
            //         t_curr = _imgs_bufl.front().header.stamp.sec * 1e9 + _imgs_bufl.front().header.stamp.nanosec;

            //         // Check if this measurement can be added to the current frame
            //         if (std::abs(t_curr*1e-9 - t_last*1e-9) > time_tolerance && !sensors.empty()) {

            //             // Create a frame with the stored sensors
            //             std::shared_ptr<isae::Frame> f = std::shared_ptr<isae::Frame>(new isae::Frame());
            //             f->init(sensors, t_last);

            //             // Add a gnss measurement if available and images in the frame
            //             if (_gnss_buf.empty() || f->getSensors().empty()) {
            //                 _pipe->_nf_queue.push(std::make_shared<NavFrame>(f));
            //                 std::stringstream().swap(msg);
            //                 msg << "Created NF (mono) " << " (" << _pipe->_nf_queue.size() << "in queue)"  << std::endl;
            //                 std::cout << msg.str();
            //             } else {
            //                 _pipe->_nf_queue.push(std::make_shared<NavFrame>(f, _gnss_buf.front()));
            //                 _gnss_buf.pop();
            //                 std::stringstream().swap(msg);
            //                 msg << "Created NF (mono | GNSS) " << " (" << _pipe->_nf_queue.size() << "in queue)"  << std::endl;
            //                 std::cout << msg.str();
            //             }

            //             sensors.clear();
            //         }

            //         image0 = getGrayImageFromMsg(_imgs_bufl.front());
            //         _imgs_bufl.pop();
            //         imgs.push_back(image0);

            //         img_sensors = _prov->createImageSensors(imgs);
            //         sensors.push_back(img_sensors.at(0));

            //         t_last = t_curr;
            //     }
            // }

            // IMU message
            // if (!_imu_buf.empty()) {
            //     std::cout << "Found IMU in buffer!" << std::endl;
            //     t_curr = _imu_buf.front().header.stamp.sec * 1e9 + _imu_buf.front().header.stamp.nanosec;
            //     t_curr -= _prov->getIMUConfig()->dt_imu_cam * 1e9;

            //     // Check if this measurement can be added to the current frame
            //     if (std::abs(t_curr*1e-9 - t_last*1e-9) > time_tolerance && !sensors.empty()) {

            //         // Create a frame with the stored sensors
            //         std::shared_ptr<isae::Frame> f = std::shared_ptr<isae::Frame>(new isae::Frame());
            //         f->init(sensors, t_last);

            //         // Add a gnss measurement if available
            //         if (_gnss_buf.empty() || f->getSensors().empty()) {
            //             _pipe->_nf_queue.push(std::make_shared<NavFrame>(f));
            //             std::cout << "Created NF (imu) " << " (" << _pipe->_nf_queue.size() << "in queue)"  << std::endl;
            //         } else {
            //             _pipe->_nf_queue.push(std::make_shared<NavFrame>(f, _gnss_buf.front()));
            //             _gnss_buf.pop();
            //             std::cout << "Created NF (imu | GNSS) " << " (" << _pipe->_nf_queue.size() << "in queue)"  << std::endl;
            //         }

            //         sensors.clear();
            //     }

            //     Eigen::Vector3d acc, gyr;
            //     _imu_mutex.lock();
            //     getImuInfoFromMsg(_imu_buf.front(), acc, gyr);
            //     _imu_buf.pop();
            //     _imu_mutex.unlock();
            //     std::shared_ptr<isae::IMU> imu_ptr = _prov->createImuSensor(acc, gyr);
            //     sensors.push_back(imu_ptr);

            //     t_last = t_curr;
            // }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        }

        std::cout << "\n Bag reader SyncProcess thread is terminating!\n";
    }

    std::shared_ptr<isae::ADataProvider> _prov;
    std::shared_ptr<Pipeline> _pipe;

    std::shared_ptr<CameraSubscriber> _cam_sub;
    std::shared_ptr<GnssSubscriber> _gnss_sub;

    std::queue<sensor_msgs::msg::Imu> _imu_buf;
    std::mutex _img_mutex;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subscription_imu;
};
