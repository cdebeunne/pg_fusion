#ifndef ROSVISUALIZER_H
#define ROSVISUALIZER_H

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "navframe.hpp"
#include "pipeline.hpp"


class RosVisualizer : public rclcpp::Node {

  public:
    RosVisualizer() : Node("pg_publisher") {
        std::cout << "\n Creation of ROS vizualizer" << std::endl;

        _pub_traj                = this->create_publisher<visualization_msgs::msg::Marker>("pg_traj", 1000);
        _pub_pose                = this->create_publisher<geometry_msgs::msg::PoseStamped>("pg_pose", 1000);
        _pub_slam                = this->create_publisher<geometry_msgs::msg::PoseStamped>("pg_slam", 1000);
        _tf_broadcaster             = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        _traj_msg.type    = visualization_msgs::msg::Marker::LINE_STRIP;
        _traj_msg.color.a = 1.0;
        _traj_msg.color.r = 0.0;
        _traj_msg.color.g = 0.0;
        _traj_msg.color.b = 1.0;
        _traj_msg.scale.x = 0.05;

    }

    void publishPose(const Eigen::Affine3d T_n_f) {

        geometry_msgs::msg::PoseStamped Tnf_msg;
        Tnf_msg.header.stamp    = rclcpp::Node::now();
        Tnf_msg.header.frame_id = "world";

        // Deal with position
        geometry_msgs::msg::Point p;
        Eigen::Vector3d tnf = T_n_f.translation();
        p.x                       = tnf.x();
        p.y                       = tnf.y();
        p.z                       = tnf.z();
        Tnf_msg.pose.position     = p;

        // Deal with orientation
        geometry_msgs::msg::Quaternion q;
        Eigen::Quaterniond eigen_q = (Eigen::Quaterniond)T_n_f.linear();
        q.x                              = eigen_q.x();
        q.y                              = eigen_q.y();
        q.z                              = eigen_q.z();
        q.w                              = eigen_q.w();
        Tnf_msg.pose.orientation         = q;

        // Publish transform
        geometry_msgs::msg::TransformStamped Tnf_tf;
        Tnf_tf.header.stamp            = rclcpp::Node::now();
        Tnf_tf.header.frame_id         = "world";
        Tnf_tf.child_frame_id          = "robot";
        Tnf_tf.transform.translation.x = tnf.x();
        Tnf_tf.transform.translation.y = tnf.y();
        Tnf_tf.transform.translation.z = tnf.z();
        Tnf_tf.transform.rotation      = Tnf_msg.pose.orientation;
        _tf_broadcaster->sendTransform(Tnf_tf);

        // publish messages
        _pub_pose->publish(Tnf_msg);
    }

    void publishFrame(const std::shared_ptr<NavFrame> frame) {

        geometry_msgs::msg::PoseStamped Twf_msg;
        Twf_msg.header.stamp    = rclcpp::Time(frame->_timestamp);
        Twf_msg.header.frame_id = "world";

        // Deal with position
        geometry_msgs::msg::Point p;
        Eigen::Vector3d twf = frame->_T_w_f.translation();
        p.x                       = twf.x();
        p.y                       = twf.y();
        p.z                       = twf.z();
        Twf_msg.pose.position     = p;

        // Deal with orientation
        geometry_msgs::msg::Quaternion q;
        Eigen::Quaterniond eigen_q = (Eigen::Quaterniond)frame->_T_w_f.linear();
        q.x                              = eigen_q.x();
        q.y                              = eigen_q.y();
        q.z                              = eigen_q.z();
        q.w                              = eigen_q.w();
        Twf_msg.pose.orientation         = q;

        // Publish transform
        geometry_msgs::msg::TransformStamped Twf_tf;
        Twf_tf.header.stamp            = rclcpp::Time(frame->_timestamp);
        Twf_tf.header.frame_id         = "world";
        Twf_tf.child_frame_id          = "slam";
        Twf_tf.transform.translation.x = twf.x();
        Twf_tf.transform.translation.y = twf.y();
        Twf_tf.transform.translation.z = twf.z();
        Twf_tf.transform.rotation      = Twf_msg.pose.orientation;
        _tf_broadcaster->sendTransform(Twf_tf);

        // publish messages
        _pub_slam->publish(Twf_msg);
    }

    void publishMap(const std::vector<std::shared_ptr<NavFrame>> nav_frames) {

        _traj_msg.header.stamp    = rclcpp::Node::now();
        _traj_msg.header.frame_id = "world";
        _traj_msg.points.clear();
        geometry_msgs::msg::Point p;

        for (auto &frame : nav_frames) {
            const Eigen::Vector3d twc = frame->_T_n_f.translation();
            p.x                       = twc.x();
            p.y                       = twc.y();
            p.z                       = twc.z();
            _traj_msg.points.push_back(p);
        }

        // publish message
        _pub_traj->publish(_traj_msg);
    }

    void runVisualizer(std::shared_ptr<Pipeline> pipe) {

        while (true) {
            
            if (!pipe->_nav_frames.empty()) {
                publishPose(pipe->_T_n_f);
                publishFrame(pipe->_nav_frames.back());
                publishMap(pipe->_nav_frames);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        }
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pub_traj;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub_pose, _pub_slam;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    visualization_msgs::msg::Marker _traj_msg;
};

#endif // ROSVISUALIZER_H