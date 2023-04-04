## Build Nodes
### Pose Controller
add_executable(pose_controller
  pid_controller.cpp
  pose_controller_node.cpp
)
target_link_libraries(pose_controller
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)
ament_target_dependencies(pose_controller
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)
rosidl_target_interfaces(pose_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Twist Accumulator
add_executable(twist_accumulator
  twist_accumulator.cpp
)
target_link_libraries(twist_accumulator

)
ament_target_dependencies(twist_accumulator
  "rclcpp"
  "geometry_msgs"
)
# rosidl_target_interfaces(twist_accumulator ${PROJECT_NAME} "rosidl_typesupport_cpp")

### Control Node
add_executable(fleet_control_node
  fleet_control_node.cpp
)
target_link_libraries(fleet_control_node
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fleet_control_node
  "rclcpp"
  "geometry_msgs"
  "edu_robot"
)
rosidl_target_interfaces(fleet_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
