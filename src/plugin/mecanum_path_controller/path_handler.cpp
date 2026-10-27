#include "edu_fleet/mecanum_path_controller/path_handler.hpp"

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

} // namespace edu_fleet::mecanum_path_controller