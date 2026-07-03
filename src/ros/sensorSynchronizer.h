#include "isaeslam/data/frame.h"
#include "isaeslam/dataproviders/adataprovider.h"

#include <mutex>
#include <thread>

#include "clockSynchronizer.hpp"
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

    /**
     * Process images (VIO-only mode) until first GNSS measurement comes in.
     * Assume that first GNSS measurement is more or less synchronized with images.
     */
    void init() {
        while (!_is_init) {
            _nf = nullptr;
            
            // Case Stereo
            if (_prov->getNCam() == 2) {
                if (!_cam_sub->emptyAny()) {
                    // reportStereoImageDetection(_msg);

                    std::shared_ptr<isae::Frame> f;
                    if (checkStereoSync(_msg)) {
                        std::vector<rclcpp::Time> t = _cam_sub->getTimeStamps();
                        _t_cam_curr = t.at(0).nanoseconds();
                        // reportTimestamp(_msg, _t_cam_curr);
                        
                        addStereoImageToSensors(_sensors);

                        // Create a VIO frame with the stored sensors
                        if (!_sensors.empty()) {
                            f = std::shared_ptr<isae::Frame>(new isae::Frame());
                            f->init(_sensors, _t_cam_curr);
                        }
                    }
                    if (f) {
                        if (_gnss_sub->empty()) {   // VIO only
                            _nf = std::make_shared<NavFrame>(f);
                            // reportNavFrameCreationStereo(_msg);
                        } else {                    // VIO + GNSS
                            std::shared_ptr<GNSSMeas> gnss_meas = _gnss_sub->getMeas();
                            if (gnss_meas) {
                                _ts_gnss_curr = gnss_meas->ts_long;
                                clock_pair_t t1t2(_ts_gnss_curr, _t_cam_curr);   
                                _clockKF->init(t1t2);

                                _nf = std::make_shared<NavFrame>(f, std::make_shared<GnssSensor>(gnss_meas, _pipe->_param->_gnss.thresh_cov));
                                if (_nf && _clockKF->_is_init) {    // Consider more conditions as needed
                                    _is_init = true;
                                    std::cout << "#### [PGSS] SensorSynchronizer is Initialized ###" << std::endl;
                                }
                                reportNavFrameCreationGNSS(_msg);
                            } else {
                                std::cerr << "[PGSS] GNSS queue is not empty, but no measurement was obtained!" << std::endl;          
                            }                
                        }
                        if (_nf && _nf->_timestamp == _t_cam_curr)
                            _pipe->_nf_queue.push(_nf);
                    }
                    _sensors.clear();
                    _t_cam_last = _t_cam_curr;
                }
            }            
        }
    }

    void sync_process() {
        std::cout << "\n[PG] Starting the measurements reader thread!\n";

        enum SyncStep { VIO, GVIO, NONE };
        while (true) {
            if (!_is_init)
                init();

            _nf = nullptr;
            // Case Stereo
            if (_prov->getNCam() == 2) {
                if (!_cam_sub->emptyAny()) {
                    // reportStereoImageDetection(_msg);
                    // _cam_sub->reportSize();

                    if (checkStereoSync(_msg)) {
                        std::vector<rclcpp::Time> t = _cam_sub->getTimeStamps();
                        _t_cam_curr = t.at(0).nanoseconds();

                        // reportTimestamp(_msg, _t_cam_curr);
                        // _gnss_sub->reportSize();
                        
                        // Check if this measurement can be added to the current frame
                            
                        // Add a gnss measurement if available and images in the frame
                        
                        SyncStep step = NONE;
                        if (_gnss_sub->empty()) {
                            step = VIO;
                        } else {                                 
                            
                            // check if the step between two GNSS measurement timestamps 
                            // is similar to the step in wall time
                            // otherwise delayed measurements can ruin the graph
                            _ts_gnss_curr = _gnss_sub->getTimeStamp();

                            // Get camera rostime for GNSS TOW
                            unsigned long long t_gnss_curr = _clockKF->asT2(_ts_gnss_curr);
                            if (_t_cam_curr > t_gnss_curr) {    // camera stamp is LATER than GNSS stamp
                                double dt = ((1e-9) * (double) (_t_cam_curr - t_gnss_curr));
                                if (dt < _t_tol_cam_gnss_s) {
                                    step = GVIO;
                                } else {
                                    // sensor sync tolerance violated
                                    // following (later) camera img will have even larger offset
                                    // --> GNSS will never have adequate match
                                    //     --> discard GNSS measurement, keep image and try again with next GNSS measurement                                        
                                    _gnss_sub->pop();
                                    reportNavFrameDiscarded(_msg, dt);
                                    step = NONE;
                                }
                            } else {                            // camera stamp is EARLIER than GNSS stamp
                                double dt = ((1e-9) * (double) (t_gnss_curr - _t_cam_curr));
                                if (dt < _t_tol_cam_gnss_s) {
                                    step = GVIO;
                                } else {
                                    // sensor sync tolerance violated
                                    // following (later) camera img may have smaller offset
                                    // --> GNSS may match later image
                                    //     --> keep GNSS measurement, send image to VIO for VIO-only step
                                    reportWaitForGNSS(_msg, dt);
                                    step = VIO;
                                }
                            }
                        }
                        std::shared_ptr<isae::Frame> f;
                        if (step == VIO || step == GVIO) {                                
                            addStereoImageToSensors(_sensors);
                            if (!checkReportEmptySensorError(_sensors, _msg)) {
                                // Create a VIO frame with the stored sensors
                                f = std::shared_ptr<isae::Frame>(new isae::Frame());
                                f->init(_sensors, _t_cam_curr);
                            }
                        }
                        switch (step)
                        {
                            case VIO:
                            {
                                if (f && !f->getSensors().empty()) {
                                    _nf = std::make_shared<NavFrame>(f);
                                }
                                // reportNavFrameCreationStereo(_msg);
                                break;
                            }
                            case GVIO:
                            {    
                                std::shared_ptr<GNSSMeas> gnss_meas = _gnss_sub->getMeas();
                                if (f && !f->getSensors().empty()) {
                                    _nf = std::make_shared<NavFrame>(
                                        f, std::make_shared<GnssSensor>(
                                            gnss_meas, _pipe->_param->_gnss.thresh_cov));
                                }
                                        
                                clock_pair_t t1t2(_ts_gnss_curr, _t_cam_curr);
                                _clockKF->propagate(_ts_gnss_curr);
                                _clockKF->update(t1t2);

                                _ts_gnss_last = _ts_gnss_curr;
                                reportNavFrameCreationGNSS(_msg);
                                break;
                            }
                            case NONE:
                                break;
                        }
                        if (_nf) {
                            _pipe->_nf_queue.push(_nf);
                        }
                        _sensors.clear();
                        _t_cam_last = _t_cam_curr;
                    }
                }
            }

                // Case mono
            // } 
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

    /**
     * Discard images from the queue if the most recent image in one queue
     * is too old w.r.t. the other queue (i.e. we lost an image somewhere)
     */
    bool checkStereoSync(std::stringstream &msg)
    {
        unsigned long long dt_tol_nanos = (unsigned long long) (1e9) * _t_tol_cam_sync_s;
        bool sync_ok = true;
        std::vector<rclcpp::Time> t = _cam_sub->getTimeStamps();
        unsigned long long time0 = t.at(0).nanoseconds();
        unsigned long long time1 = t.at(1).nanoseconds();

        // sync tolerance
        if (time0 < time1 - dt_tol_nanos) {
            _cam_sub->pop(0);
            reportStereoImgSyncError(0, time0, time1, msg);
            sync_ok = false;
        } else if (time0 > time1 + dt_tol_nanos) {
            _cam_sub->pop(1);
            reportStereoImgSyncError(1, time0, time1, msg);
            sync_ok = false;
        } 
        return sync_ok;
    }

    void reportNavFrameDiscarded(std::stringstream &msg, double dt)
    {
        std::stringstream().swap(msg);
        msg << "############################################################" << std::endl;
        msg << "[PGSS] Discarded NF (stereo | GNSS) " << " (" << dt << " s)"  << std::endl;
        msg << "############################################################" << std::endl;
        std::cout << msg.str();
    }

    void reportWaitForGNSS(std::stringstream &msg, double dt)
    {
        std::stringstream().swap(msg);
        // msg << "############################################################" << std::endl;
        msg << "[PGSS] Waiting ... (stereo | GNSS) " << " (" << dt << " s)"  << std::endl;
        // msg << "############################################################" << std::endl;
        std::cout << msg.str();
    }

    void reportStereoImgSyncError(uint cam, unsigned long long time0, unsigned long long time1, std::stringstream &msg)
    {
        std::stringstream().swap(msg);
        msg << "\n Throw img" << cam << " -- Sync error : " << (time0 - time1) << "\n";
        std::cout << msg.str();
    }

    void reportTimestamp(std::stringstream &msg, unsigned long long t_curr)
    {
        std::stringstream().swap(msg);
        msg << "[PGSS] Frame timestamp: " << t_curr << std::endl;
        std::cout << msg.str();
    }

    void reportStereoImageDetection(std::stringstream &msg)
    {
        std::stringstream().swap(msg);
        msg << "[PGSS] Found STEREO image in buffer! " 
            << "(" << _cam_sub->size(0) << "|" << _cam_sub->size(1) << ")" << std::endl;
        std::cout << msg.str();
    }

    void reportNavFrameCreationStereo(std::stringstream &msg)
    {
        std::stringstream().swap(msg);
        msg << "[PGSS] Created NF (stereo) " << "( " << _pipe->_nf_queue.size() << "in queue)" << std::endl;
        std::cout << msg.str();
    }

    void reportNavFrameCreationGNSS(std::stringstream &msg)
    {
        std::stringstream().swap(msg);
        msg << "[PGSS] Created NF (stereo | GNSS) " << " (" << _pipe->_nf_queue.size() << " in queue)"  << std::endl;
        std::cout << msg.str();
    }

    bool checkReportEmptySensorError(std::vector<std::shared_ptr<isae::ASensor>> &sensors, std::stringstream &msg)
    {
        bool isEmpty = sensors.empty();
        if (isEmpty) {
            std::stringstream().swap(msg);
            msg << "[PGSS] Sensors empty!" << std::endl;
            std::cout << msg.str();
        }
        return isEmpty;
    }

    void checkReportTimeTolError(unsigned long long t_curr, unsigned long long t_last, double time_tolerance, std::stringstream &msg)
    {
        if (std::abs(t_curr*1e-9 - t_last*1e-9) <= time_tolerance) {
            std::stringstream().swap(msg);
            msg << "[PGSS] Time tolerance violated: " << std::abs(t_curr*1e-9 - t_last*1e-9) << " <= " << time_tolerance << std::endl;
            std::cout << msg.str();
        }
    }

    void addStereoImageToSensors(std::vector<std::shared_ptr<isae::ASensor>> &sensors)
    {
        cv::Mat image0, image1;
        std::vector<cv::Mat> imgs;
        image0 = _cam_sub->getGrayImageMono(0);
        if (!image0.empty())
            imgs.push_back(image0);

        image1 = _cam_sub->getGrayImageMono(1);
        if (!image1.empty())
            imgs.push_back(image1);

        std::vector<std::shared_ptr<isae::ImageSensor>> img_sensors;
        if (!imgs.empty())
            img_sensors = _prov->createImageSensors(imgs);

        if (img_sensors.size() == 2) {
            sensors.push_back(img_sensors.at(0));
            sensors.push_back(img_sensors.at(1));
        }
    }

    std::shared_ptr<isae::ADataProvider> _prov;
    std::shared_ptr<Pipeline> _pipe;

    std::shared_ptr<CameraSubscriber> _cam_sub;
    std::shared_ptr<GnssSubscriber> _gnss_sub;

    std::queue<sensor_msgs::msg::Imu> _imu_buf;
    std::mutex _img_mutex;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subscription_imu;

    bool _is_init = false;

    protected:
        std::vector<std::shared_ptr<isae::ASensor>> _sensors;
        const double _t_tol_cam_sync_s      = 0.025;   // TODO add this as a parameter of the yaml
        const double _t_tol_cam_gnss_s      = 0.03;       // TODO add this as a parameter of the yaml
        unsigned long long _t_cam_last      = 0;
        unsigned long long _t_cam_curr      = 0;
        unsigned long long _ts_gnss_last    = 0;
        unsigned long long _ts_gnss_curr    = 0;

        std::stringstream _msg;

        std::shared_ptr<NavFrame> _nf;

        std::shared_ptr<ClockSynchronizer> _clockKF = std::make_shared<ClockSynchronizer>();

};
