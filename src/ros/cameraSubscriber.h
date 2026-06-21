#ifndef CAMERASUBSCRIBER_H
#define CAMERASUBSCRIBER_H

#include "isaeslam/data/frame.h"
#include "isaeslam/dataproviders/adataprovider.h"

#include <mutex>
#include <thread>

#include "pipeline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>

typedef std::vector<std::string> topic_names;
// template <typename T>
typedef std::queue<sensor_msgs::msg::Image> img_queue_t;
typedef std::vector<std::shared_ptr<img_queue_t>> img_readers_t;

class CameraSubscriber : public rclcpp::Node {

  public:
    CameraSubscriber(std::shared_ptr<isae::ADataProvider> prov)
        : Node("camera_subscriber"), _prov(prov) {

        for (uint ncam = 0; ncam < _prov->getNCam(); ncam++ ) {
            std::string cam_topic = _prov->getCamConfigs().at(ncam)->ros_topic;
            _imgs_topics.push_back(cam_topic);

            std::shared_ptr<img_queue_t> cam_queue = std::make_shared<img_queue_t>();;
            _imgs_buf.push_back(cam_queue);

            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
            if (ncam == 0) 
            {
                subscription = this->create_subscription<sensor_msgs::msg::Image>(
                    cam_topic, 10, std::bind(&CameraSubscriber::subFirstImage, this, std::placeholders::_1));
            }
            else if (ncam == 1)
            {
                subscription = this->create_subscription<sensor_msgs::msg::Image>(
                    cam_topic, 10, std::bind(&CameraSubscriber::subSecondImage, this, std::placeholders::_1));
            }
            _subscriptions.push_back(subscription);
            std::cout << "[PG] Added camera subscription (" << ncam << ") " << cam_topic << std::endl;
        }
    }

    void subFirstImage(const sensor_msgs::msg::Image &img_msg) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        subImage(img_msg, 0);
    }

    void subSecondImage(const sensor_msgs::msg::Image &img_msg) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        subImage(img_msg, 1);
    }

    void pop(uint cam) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        if (_imgs_buf.size() > cam && !_imgs_buf.at(cam)->empty())
            _imgs_buf.at(cam)->pop();
    }

    bool emptyAll() {        
        std::lock_guard<std::mutex> lock(_img_mutex);
        bool isEmpty = true;
        for (auto buf : _imgs_buf) {
            isEmpty &= buf->empty();
        }
        return isEmpty;
    }
    bool emptyAny() {        
        std::lock_guard<std::mutex> lock(_img_mutex);
        bool isEmpty = false;
        for (auto buf : _imgs_buf) {
            isEmpty |= buf->empty();
        }
        return isEmpty;
    }

    bool empty(uint cam) {        
        std::lock_guard<std::mutex> lock(_img_mutex);
        return _imgs_buf.at(cam)->empty();;
    }
    
    std::size_t size(uint cam) {
        std::lock_guard<std::mutex> lock(_img_mutex);        
        return _imgs_buf.at(cam)->size();
    } 


    std::vector<rclcpp::Time> getTimeStamps() {
        std::lock_guard<std::mutex> lock(_img_mutex);
        std::vector<rclcpp::Time> t;
        for (auto buf : _imgs_buf) {
            if (!buf->empty()) {
                 t.push_back(buf->front().header.stamp);
            } else {
                 t.push_back(rclcpp::Time(0));
            }
        }
        return t;
    }

    cv::Mat getImageMono(uint cam) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        cv::Mat img;
        if (_imgs_buf.size() > cam && !_imgs_buf.at(cam)->empty()) {
            img = getImageFromMsg(_imgs_buf.at(cam)->front());
            _imgs_buf.at(cam)->pop();
        }
        return img;
    }

    cv::Mat getGrayImageMono(uint cam) {
        std::lock_guard<std::mutex> lock(_img_mutex);
        cv::Mat img;
        if (_imgs_buf.size() > cam && !_imgs_buf.at(cam)->empty()) {
            img = getGrayImageFromMsg(_imgs_buf.at(cam)->front());
            _imgs_buf.at(cam)->pop();
        }
        return img;
    }


    cv::Mat getImageFromMsg(const sensor_msgs::msg::Image &img_msg) {
        // Get and prepare images
        cv_bridge::CvImagePtr ptr;
        try {
            ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            std::cout << "\n\n\ncv_bridge exeception: %s\n\n\n" << e.what() << std::endl;
        }

        return ptr->image;
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

    void sync_process() {
        std::cout << "\n [PG] Starting the camera reader thread!\n";
        std::cout << "[PG] Cam reader has " << _imgs_buf.size() << " cameras" << std::endl;

        std::vector<std::shared_ptr<isae::ASensor>> sensors;
        unsigned long long t_lastl         = 0;
        unsigned long long t_lastr         = 0;
        unsigned long long t_curr         = 0;
        // unsigned long long ts_cam_lastl = 0;
        // unsigned long long ts_cam_lastr = 0;

        while (true) {
            std::lock_guard<std::mutex> lock(_img_mutex);
            if (_imgs_buf.size() > 0 && !_imgs_buf.at(0)->empty()) {
                t_curr = this->now().nanoseconds();
                std::stringstream msg;
                msg << "[CAMERA SUB] " << "Found image (prim) " << " (" << _imgs_buf.at(0)->size() << " in queue)"  << std::endl;                
                msg << "[CAMERA SUB] Current time: " <<  t_curr << std::endl;
                msg << "[CAMERA SUB] " << (t_curr - t_lastl)*1e-9 << " s elapsed since last prim image" << std::endl;
                std::cout << msg.str();
                t_lastl = t_curr;
            }
            if (_imgs_buf.size() > 1 && !_imgs_buf.at(1)->empty()) {
                t_curr = this->now().nanoseconds();
                std::stringstream msg;
                msg << "[CAMERA SUB] " << "Found image (scnd) " << " (" << _imgs_buf.at(1)->size() << " in queue)"  << std::endl;                
                msg << "[CAMERA SUB] Current time: " <<  t_curr << std::endl;
                msg << "[CAMERA SUB] " << (t_curr - t_lastr)*1e-9 << " s elapsed since last scnd image" << std::endl;
                std::cout << msg.str();
                t_lastr = t_curr;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << "\n Bag reader SyncProcess thread is terminating!\n";
    }

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> _subscriptions;

  protected:

    const uint __max_buf_len = 100;

    std::shared_ptr<isae::ADataProvider> _prov;
    std::string _cam_l_topic, _cam_r_topic;

    std::queue<sensor_msgs::msg::Image> _imgs_bufprim, _imgs_bufscnd;
    topic_names _imgs_topics;
    img_readers_t _imgs_buf;
    std::mutex _img_mutex;

    void subImage(const sensor_msgs::msg::Image &img_msg, int ncam) { 
        std::stringstream  msg;
        msg << "[CAMERA SUB] Got image! " << ncam << std::endl;   
        // std::cout << msg.str();
        std::stringstream().swap(msg);   

        if (_imgs_buf.size() <= ncam) {
            std::cerr << "[CAMERA SUB] No queue available for cam " << ncam << std::endl;
            return;
        }

        if (_imgs_buf.at(ncam)->size() > __max_buf_len) {
            _imgs_buf.at(ncam)->pop();
            msg << "[CAMERA SUB] Warning: Image buffer full! Discarded image." << std::endl;
        }

        _imgs_buf.at(ncam)->push(img_msg);

        msg << "[CAMERA SUB] Current time: " <<  this->now().nanoseconds() << std::endl;
        msg << "[CAMERA SUB] " << _imgs_topics.at(ncam) << " Buffer contains " 
            << _imgs_buf.at(ncam)->size() << " elements." << std::endl;
        // std::cout << msg.str();
    }
};

#endif