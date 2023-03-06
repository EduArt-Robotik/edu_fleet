## Build Nodes
### Fleet Control
add_executable(pose_controller
  # fleet_control_node.cpp
  pid_controller.cpp
  pose_controller_node.cpp
)
target_link_libraries(pose_controller
  Eigen3::Eigen
)
ament_target_dependencies(pose_controller
  "rclcpp"
  "geometry_msgs"
)
# rosidl_target_interfaces(pose_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
