## Build Nodes
### Pose Controller
add_executable(pose_controller
  pose_controller_node.cpp
)
target_link_libraries(pose_controller
  Eigen3::Eigen
  "${cpp_typesupport_target}"
  eduart-fleet-controller
)
ament_target_dependencies(pose_controller
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "aruco_opencv_msgs"
  "edu_robot"
)

### Line Controller
### Optional
if (${sick_lidar_localization_FOUND})
  message(STATUS "Building Line Controller for Sick Line Follower")

  add_executable(sick_line_controller_node
    sick_line_controller_node.cpp
  )
  target_link_libraries(sick_line_controller_node
    eduart-fleet-controller
  )
  ament_target_dependencies(sick_line_controller_node
    "rclcpp"
    "geometry_msgs"
    "sick_lidar_localization"
  )

  add_executable(sick_line_navigation_node
    sick_line_navigation_node.cpp
  )
  target_link_libraries(sick_line_navigation_node

  )
  ament_target_dependencies(sick_line_navigation_node
    "rclcpp"
    "geometry_msgs"
    "sick_lidar_localization"
  )

endif()

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

### Control Node
add_executable(fleet_control_node
  fleet_control_node.cpp
)
target_link_libraries(fleet_control_node
  eduart-fleet-sensor-model  
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fleet_control_node
  "rclcpp"
  "geometry_msgs"
  "edu_robot"
  "visualization_msgs"
)

### Fleet Localization
add_executable(fleet_localization_node
  fleet_localization_node.cpp
)
target_link_libraries(fleet_localization_node
  Eigen3::Eigen
  eduart-fleet-kalman-filter
  eduart-fleet-sensor-model
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fleet_localization_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "edu_robot"
  "std_srvs"
)

### Robot Localization
add_executable(robot_localization_node
  robot_localization_node.cpp
)
target_link_libraries(robot_localization_node
  Eigen3::Eigen
  eduart-fleet-kalman-filter
  eduart-fleet-sensor-model
  eduart-fleet-transform
  "${cpp_typesupport_target}"
)
ament_target_dependencies(robot_localization_node
  "rclcpp"
  "geometry_msgs"
  "tf2_sensor_msgs"
  "nav_msgs"
  "edu_robot"
  "std_srvs"
)

### Collision Avoidance LiDAR
add_executable(collision_avoidance_lidar_node
  collision_avoidance_lidar_node.cpp
)
target_link_libraries(collision_avoidance_lidar_node

)
ament_target_dependencies(collision_avoidance_lidar_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
)