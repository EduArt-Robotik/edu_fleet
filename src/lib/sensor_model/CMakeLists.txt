add_library(eduart-fleet-sensor-model
  sensor_model.cpp
  message_converting.cpp
)
target_link_libraries(eduart-fleet-sensor-model
  eduart-fleet-kalman-filter
)
target_compile_features(eduart-fleet-sensor-model PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(eduart-fleet-sensor-model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(eduart-fleet-sensor-model
  "rclcpp"
  "sensor_msgs"
)
