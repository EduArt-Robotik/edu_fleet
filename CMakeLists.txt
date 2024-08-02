cmake_minimum_required(VERSION 3.8)
project(edu_fleet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++1z)# -O2)
endif()

## Dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(edu_robot REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(std_srvs REQUIRED)

## Build Defined ROS Messages
rosidl_generate_interfaces(${PROJECT_NAME}
# Messages
# Services
  "srv/GetTransform.srv"
  DEPENDENCIES
  "edu_robot"
)
# Create a target that can be used by all buildiung targets when the custom message are required for them.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Building Library and Nodes
add_subdirectory(src/lib)
add_subdirectory(src/node)

## Installing of Content
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY
    launch
    parameter
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS
    # nodes
    pose_controller
    fleet_control_node
    twist_accumulator
    fleet_localization_node
    robot_localization_node
    # libraries
    eduart-fleet-sensor-model
    eduart-fleet-kalman-filter
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(src/test)
endif()

ament_package()
