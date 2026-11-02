#include "edu_fleet/mecanum_path_controller/path_handler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cstddef>
#include <rclcpp/logging.hpp>

namespace edu_fleet::mecanum_path_controller {

PathHandler::PathHandler(tf2::Duration transform_tolerance,
                         std::shared_ptr<tf2_ros::Buffer> tf,
                         double waypoint_distance)
    : transform_tolerance_(transform_tolerance), tf_(tf),
      waypoint_distance_(waypoint_distance) {};

geometry_msgs::msg::PoseStamped
PathHandler::getLocalGoal(const geometry_msgs::msg::PoseStamped &global_pose) {
    if(current_index_ == global_plan_.poses.size() - 1)
    {
        return global_plan_.poses[current_index_];
    }

    // due to the check above, ther is always one more pose on the path ahead
    auto distance = std::hypot(global_pose.pose.position.x - global_plan_.poses[current_index_].pose.position.x,
    global_pose.pose.position.y - global_plan_.poses[current_index_].pose.position.y);

    if (distance < waypoint_distance_)
    {
        ++current_index_;
    }

    return global_plan_.poses[current_index_];
}

geometry_msgs::msg::PoseStamped PathHandler::getGlobalGoal()
{
    if(global_plan_.poses.size() > 0)
    {
        return global_plan_.poses[global_plan_.poses.size() - 1];
    }
    return geometry_msgs::msg::PoseStamped();
}

double PathHandler::getMaxAngle(size_t lookahead, size_t lookback)
{
    const auto & poses = global_plan_.poses;
    const size_t path_size = static_cast<size_t>(poses.size());

     // Include up to two points before the current index
    const int start_index = current_index_ - std::min(current_index_, lookback);

    // Do not access points outside the path
    const int end_index = std::min(path_size - 2, current_index_ + lookahead);

    double max_angle = 0.0;

    for (int i = start_index; i < end_index; ++i)
    {
        // Calculate the direction vectors of the two consecutive segments
        const double dx1 = poses[i + 1].pose.position.x - poses[i].pose.position.x;
        const double dy1 = poses[i + 1].pose.position.y - poses[i].pose.position.y;

        const double dx2 = poses[i + 2].pose.position.x - poses[i + 1].pose.position.x;
        const double dy2 = poses[i + 2].pose.position.y - poses[i + 1].pose.position.y;

        // Calculate the angle between the segments using the dot product
        const double dot_product = dx1 * dx2 + dy1 * dy2;
        const double cross_product = dx1 * dy2 - dy1 * dx2;

        const double angle = std::abs(std::atan2(cross_product, dot_product));

        // Store the maximum angle
        max_angle = std::max(max_angle, angle);
    }

    return max_angle;
}



} // namespace edu_fleet::mecanum_path_controller