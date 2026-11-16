#include "robot_status_publisher.hpp"
#include "edu_robot/msg/robot_status_report.hpp"
#include "edu_robot/msg/state.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

#include <format>

using namespace std::chrono_literals;

namespace eduart::fleet {

    RobotStatusPublisher::Parameter RobotStatusPublisher::get_parameter(rclcpp::Node &ros_node)
    {
        Parameter params;

        ros_node.declare_parameter<std::string>("topic_name", params.topic_name);
        ros_node.declare_parameter<std::string>("map_frame", params.map_frame);
        ros_node.declare_parameter<std::string>("base_frame", params.base_frame);
        ros_node.declare_parameter<std::string>("sensor_frame", params.sensor_frame);
        ros_node.declare_parameter<std::string>("state_sub_topic", params.state_sub_topic);

        params.topic_name = ros_node.get_parameter("topic_name").as_string();
        params.map_frame = ros_node.get_parameter("map_frame").as_string();
        params.base_frame = ros_node.get_parameter("base_frame").as_string();
        params.sensor_frame = ros_node.get_parameter("sensor_frame").as_string();
        params.state_sub_topic = ros_node.get_parameter("state_sub_topic").as_string();

        return params;
    }

    RobotStatusPublisher::RobotStatusPublisher() 
    : rclcpp::Node("robot_state_publisher"),
    last_state_(5),
    param_(get_parameter(*this))
    {
        publisher_ = create_publisher<std_msgs::msg::String>(
            param_.topic_name, 
            rclcpp::QoS(1).reliable()
        );

        state_subscription_ = create_subscription<edu_robot::msg::RobotStatusReport>(
            param_.state_sub_topic,
            rclcpp::QoS(1).best_effort(),
            [this](const edu_robot::msg::RobotStatusReport::SharedPtr msg) -> void
            {last_state_ = msg->robot_state.state.value;}
        );

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        register_status_functions();

        publish_timer_ = create_wall_timer(
            1s,
            std::bind(&RobotStatusPublisher::timer_callback, this)
        );
    }

    void RobotStatusPublisher::timer_callback()
    {
        bool all_conditions = true;
        std::string complete_status_report = "";

        for(auto function : status_functions_)
        {
            bool condition_success;
            std::string status_report;

            function(condition_success, status_report);

            if(condition_success == false)
            {
                all_conditions = false;
                complete_status_report += (status_report + ";");
            }
        }

        std_msgs::msg::String msg;
        if(all_conditions)
        {
            msg.data = "ready";
        }
        else
        {
            msg.data = complete_status_report;
        }

        publisher_->publish(msg);
    }

    void RobotStatusPublisher::register_status_functions()
    {
        status_functions_.push_back(std::bind(
            &RobotStatusPublisher::map_base_tf_available,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));

        status_functions_.push_back(std::bind(
            &RobotStatusPublisher::base_sensor_tf_available,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));

        status_functions_.push_back(std::bind(
            &RobotStatusPublisher::last_state_okay,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));
    }

    void RobotStatusPublisher::map_base_tf_available(bool &success, std::string &report)
    {
        success = tf_buffer_->canTransform(
            param_.map_frame,
            param_.base_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1)
        );

        // set status report if transform is not available
        if(success == false)
        {
            RCLCPP_WARN(this->get_logger(), "unable to transform from frame %s to frame %s", param_.map_frame.c_str(), param_.base_frame.c_str());
            report = "localization not available";
        }
    }
    void RobotStatusPublisher::base_sensor_tf_available(bool &success, std::string &report)
    {
        success = tf_buffer_->canTransform(
            param_.base_frame,
            param_.sensor_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1)
        );

        // set status report if transform is not available
        if(success == false)
        {
            RCLCPP_WARN(this->get_logger(), "unable to transform from frame %s to frame %s", param_.base_frame.c_str(), param_.sensor_frame.c_str());
            report = "robot not present";
        }
    }

    void RobotStatusPublisher::last_state_okay(bool &success, std::string &report)
    {
        if (last_state_ == edu_robot::msg::State::OK)
        {
            success = true;
        }
        else if (last_state_ == edu_robot::msg::State::LOW_BATTERY)
        {
            success = false;
            report = "battery low";
        }
        else
        {
            success = false;
            report = "status return of robot is invalid";
        }

    }

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<eduart::fleet::RobotStatusPublisher>()
    );
    rclcpp::shutdown();
}