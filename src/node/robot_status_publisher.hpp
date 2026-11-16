/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Philipp Fischer (philipp.fischer@eduart-robotik.com)
 */
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <vector>

#include "edu_robot/msg/robot_status_report.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace eduart::fleet {

    using StatusFunction = std::function<void(bool&, std::string&)>;

    class RobotStatusPublisher : public rclcpp::Node
    {
        public:
        struct Parameter {
            std::string topic_name = "robot_status";
            std::string map_frame = "map";
            std::string base_frame = "base_footprint";
            std::string sensor_frame = "base_link";
            std::string state_sub_topic = "status_report";
        };

        RobotStatusPublisher();
        ~RobotStatusPublisher() = default;

        static Parameter get_parameter(rclcpp::Node& ros_node);

        private:
        void timer_callback();
        void register_status_functions();

        void map_base_tf_available(bool & success, std::string& report);
        void base_sensor_tf_available(bool & success, std::string & report);
        void last_state_okay(bool & success, std::string & report);

        // member attribute required for check-functions
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Subscription<edu_robot::msg::RobotStatusReport>::SharedPtr state_subscription_;
        uint8_t last_state_;


        Parameter param_;
        std::vector<StatusFunction> status_functions_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    };
}