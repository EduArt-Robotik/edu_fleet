add_library(eduard-fleet-mecanum-path-controller SHARED
  mecanum_path_controller/mecanum_controller.cpp
  mecanum_path_controller/path_handler.cpp
)

target_compile_features(eduard-fleet-mecanum-path-controller PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
target_include_directories(eduard-fleet-mecanum-path-controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(eduard-fleet-mecanum-path-controller
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_core
  tf2
)
